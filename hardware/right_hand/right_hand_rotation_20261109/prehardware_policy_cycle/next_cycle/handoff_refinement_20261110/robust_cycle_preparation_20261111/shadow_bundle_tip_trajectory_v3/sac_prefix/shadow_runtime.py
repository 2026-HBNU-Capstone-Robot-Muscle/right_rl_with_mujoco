"""NumPy-only transactional shadow controller. This module cannot send motors.

Input positions must refer to the training palm sensor point in the declared
simulation frame. A physical camera registration is not supplied by this module.
"""
from pathlib import Path
import hashlib
import json
import numpy as np

SIM_IDS=np.array((8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1))
TICK=2*np.pi/4096


def vector(value,n,name):
    value=np.asarray(value,float)
    if value.shape!=(n,) or not np.isfinite(value).all():raise ValueError(name+' must contain finite values')
    return value


def quaternion(value):
    q=vector(value,4,'quaternion');length=np.linalg.norm(q)
    if length<1e-8:raise ValueError('Zero quaternion')
    return q/length


def multiply(a,b):
    w,x,y,z=a;v,i,j,k=b
    return np.array([w*v-x*i-y*j-z*k,w*i+x*v+y*k-z*j,w*j-x*k+y*v+z*i,w*k+x*j-y*i+z*v])


def matrix(q):
    w,x,y,z=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])


class ShadowController:
    def __init__(self,bundle):
        self.folder=Path(bundle);self.meta=json.loads((self.folder/'bundle.json').read_text())
        if self.meta.get('hardware_execution_allowed') is not False:raise ValueError('Only shadow bundles supported')
        for name,digest in self.meta['files_sha256'].items():
            if hashlib.sha256((self.folder/name).read_bytes()).hexdigest()!=digest:raise ValueError('Bundle file changed: '+name)
        with np.load(self.folder/'actor.npz',allow_pickle=False) as a:
            self.mean=a['mean'];self.std=a['std'];self.layers=[(a[f'w{i}'],a[f'b{i}']) for i in range(3)]
        with np.load(self.folder/'reference.npz',allow_pickle=False) as r:
            self.goals=r['goals'];self.start=r['start'];self.low=r['lower'];self.high=r['upper'];self.goal_quat=r['goal_quat']
        if self.mean.shape!=(110,) or self.std.shape!=(110,) or np.any(self.std<=0):raise ValueError('Actor normalization contract')
        self.joint_history=np.zeros((3,16));self.position_history=np.zeros((3,3));self.orientation_history=np.zeros((3,6))
        self.last_action=np.zeros(16);self.samples=0;self.sequence=-1;self.phase=0
        self.last_time=None;self.last_commit_time=None;self.last_goal=None;self.pending=None

    @staticmethod
    def to_sim(ticks):
        return (vector(ticks,16,'motor ticks')[SIM_IDS-1]-2048)*TICK

    @staticmethod
    def to_motor(q):
        result=np.empty(16,dtype=int);result[SIM_IDS-1]=np.rint(q/TICK+2048).astype(int);return result

    def _validate(self,sample):
        if sample.get('frame_id')!='leap_sim_world' or sample.get('palm_point')!='policy_palm_position':
            raise ValueError('Explicit training frame and palm sensor reference required')
        seq=sample['sequence'];now=float(sample['sample_time']);pose=float(sample['pose_time'])
        if not isinstance(seq,(int,np.integer)) or seq<=self.sequence:raise ValueError('Fresh sample sequence required')
        if not np.isfinite([now,pose]).all() or pose>now or now-pose>.15:raise ValueError('Cube pose is stale or time is invalid')
        if self.last_time is not None and now-self.last_time<.075-1e-8:raise ValueError('Control interval must be at least75ms')
        measured=vector(sample['measured_ticks'],16,'measured ticks')
        command=vector(sample['commanded_ticks'],16,'commanded ticks')
        if np.any(command<self.low) or np.any(command>self.high):raise ValueError('Command outside native range')
        if self.last_goal is not None and not np.array_equal(command,self.last_goal):raise ValueError('Command acknowledgement differs from committed goal')
        palm=vector(sample['palm_position'],3,'palm position');cube=vector(sample['cube_position'],3,'cube position')
        q=quaternion(sample['cube_quaternion']);return seq,now,measured,command,palm,cube,q

    def propose(self,sample):
        if self.pending is not None:raise ValueError('Commit the pending shadow goal before the next proposal')
        seq,now,measured,command,palm,cube,q=self._validate(sample)
        qm=self.to_sim(measured);qc=self.to_sim(command)
        goal=quaternion(self.goal_quat);delta=quaternion(multiply(q,goal*np.array([1,-1,-1,-1])))
        for history,value in ((self.joint_history,qm-qc),(self.position_history,palm-cube),
                              (self.orientation_history,matrix(delta).ravel()[3:])):
            history[1:]=history[:-1].copy();history[0]=value
        self.samples+=1;self.sequence=seq;self.last_time=now
        if self.samples<3:return {'state':'warming','fresh_samples':self.samples,'motor_commands_sent':0}
        obs=np.r_[qm,self.joint_history.ravel(),self.position_history.ravel(),self.orientation_history.ravel(),
                   self.last_action,2*np.arcsin(min(np.linalg.norm(delta[1:]),1.)),
                   min(self.phase/max(1,len(self.goals)-1),1.),float(self.phase>=len(self.goals)-134)].astype(np.float32)
        x=(obs-self.mean)/self.std
        for i,(w,b) in enumerate(self.layers):
            x=x@w+b
            if i<2:x=np.maximum(x,0)
        action=np.tanh(x[:16]);nominal=self.goals[min(self.phase,len(self.goals)-1)]
        proposal=self.to_motor(self.to_sim(nominal)+self.meta['residual_rad']*action)
        executed=command.astype(int)+np.clip(proposal-command.astype(int),-2,2)
        if np.any(executed<self.low) or np.any(executed>self.high):raise ValueError('Proposed goal outside native range')
        self.pending={'sequence':seq,'goal_ticks':executed.copy(),'before':command.copy(),'time':now}
        return {'state':'shadow_hold' if self.phase>=len(self.goals) else 'shadow_rotation',
                'sequence':seq,'reference_index':self.phase,'goal_ticks':executed.tolist(),
                'actor_action':action.tolist(),'motor_commands_sent':0,'next_rotation_ready':False}

    def commit_shadow(self,sequence,goal_ticks):
        if self.pending is None or sequence!=self.pending['sequence']:raise ValueError('Pending sequence mismatch')
        goal=vector(goal_ticks,16,'acknowledged goal')
        if not np.array_equal(goal,self.pending['goal_ticks']):raise ValueError('Acknowledgement changed the goal')
        self.last_action=np.clip((self.to_sim(goal)-self.to_sim(self.pending['before']))/.2,-1,1)
        self.last_goal=goal.copy();self.last_commit_time=self.pending['time'];self.phase+=1;self.pending=None
