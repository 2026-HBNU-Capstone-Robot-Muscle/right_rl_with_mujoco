"""NumPy-only first-turn stages. No motor, serial, camera or ROS interface."""
import numpy as np
from shadow_runtime import ShadowController,quaternion,multiply,matrix,TICK


class StagedShadowController(ShadowController):
    def __init__(self,bundle):
        super().__init__(bundle)
        if self.meta.get('complete_two_turn_cycle_validated') is not False:
            raise ValueError('This bundle covers only the first turn')
        if self.meta.get('acquisition_metric_is_global_clearance_certificate') is not False:
            raise ValueError('Acquisition screening must not be presented as a clearance certificate')
        with np.load(self.folder/'correction_actor.npz',allow_pickle=False) as data:
            self.cmean=data['mean'];self.cstd=data['std']
            self.clayers=[(data[f'w{i}'],data[f'b{i}']) for i in range(3)]
        with np.load(self.folder/'staging.npz',allow_pickle=False) as data:
            self.station=data['common_station'];self.up=data['up']
        self.prefix_steps=int(self.meta['prefix_steps']);self.acquire_samples=int(self.meta['acquire_samples'])
        self.correction_initialized=False;self.acquired=0;self.mode='rotation'

    def propose(self,sample):
        if self.phase<self.prefix_steps:
            result=super().propose(sample)
            if self.pending is not None:self.pending['stage']='rotation'
            result['stage']='rotation';return result
        if self.pending is not None:raise ValueError('Commit the pending shadow proposal first')
        seq,now,measured,command,palm,cube,q=self._validate(sample)
        screening=sample.get('upper_sweep_acquisition',{})
        if screening.get('model_sha256')!=self.meta['model_sha256'] or screening.get('sequence')!=seq:
            raise ValueError('Fresh acquisition screening for the same model and sample is required')
        if screening.get('method')!='slab_clipped_radial_screen':
            raise ValueError('Explicit slab/radial acquisition metric required; it is not a clearance certificate')
        gap=float(screening.get('screen_mm',float('nan')))
        if not np.isfinite(gap):raise ValueError('Finite acquisition screening metric required')
        qm=self.to_sim(measured);qc=self.to_sim(command);goal=quaternion(self.goal_quat)
        delta=quaternion(multiply(q,goal*np.array([1,-1,-1,-1])))
        orientation=matrix(delta).ravel()[3:];error=float(2*np.arcsin(min(np.linalg.norm(delta[1:]),1.)))
        tilt=float(np.degrees(np.arccos(np.clip((-matrix(q)[:,1])@self.up,-1,1))))
        qualifies=error<np.radians(1.5) and tilt<5 and np.linalg.norm(cube-self.station)<.002 and gap>.3
        initialized=self.correction_initialized
        observed_count=(self.acquired+1 if qualifies else 0) if initialized else 0
        stage='hold' if self.mode=='hold' or observed_count>=self.acquire_samples else 'correction'
        joint=self.joint_history.copy();position=self.position_history.copy();rotation=self.orientation_history.copy()
        previous_action=self.last_action.copy()
        if not initialized:
            joint[:]=qm-qc;position[:]=palm-cube;rotation[:]=orientation
            previous_action[:]=0
        else:
            for history,value in ((joint,qm-qc),(position,palm-cube),(rotation,orientation)):
                history[1:]=history[:-1].copy();history[0]=value
        obs=np.r_[qm,joint.ravel(),position.ravel(),rotation.ravel(),previous_action,error,1.,min(observed_count/54.,1.)].astype(np.float32)
        if stage=='hold':action=np.zeros(16);proposed=command.astype(int)
        else:
            x=(obs-self.cmean)/self.cstd
            for i,(w,b) in enumerate(self.clayers):
                x=x@w+b
                if i<2:x=np.maximum(x,0)
            action=np.tanh(x[:16]);proposed=self.to_motor(qc+2*TICK*action)
        if np.max(np.abs(proposed-command))>2:raise ValueError('Increment exceeds two ticks')
        if np.any(proposed<self.low) or np.any(proposed>self.high):raise ValueError('Correction proposed an out-of-range goal')
        self.joint_history[:]=joint;self.position_history[:]=position;self.orientation_history[:]=rotation
        self.sequence=seq;self.last_time=now;self.samples+=1
        self.pending=dict(sequence=seq,goal_ticks=proposed.copy(),before=command.copy(),time=now,
                          stage=stage,observed_count=observed_count)
        return dict(state='shadow_'+stage,stage=stage,sequence=seq,goal_ticks=proposed.tolist(),
                    actor_action=action.tolist(),motor_commands_sent=0,next_rotation_ready=False,
                    fixed_position_hold=stage=='hold')

    def commit_shadow(self,sequence,goal_ticks):
        if self.pending is None:raise ValueError('No pending proposal')
        stage=self.pending['stage'];count=self.pending.get('observed_count',0)
        super().commit_shadow(sequence,goal_ticks)
        if stage!='rotation':
            self.correction_initialized=True;self.acquired=count
        self.mode=stage

    def next_rotation_capability(self):
        return dict(ready=False,reason='Second rotation and full cycle have not passed validation',
                    current_stage=self.mode,motor_commands_sent=0,changes_hold_command=False)


class AdmissibleShadowController(StagedShadowController):
    """Research variant selecting nearest legal goals inside original limits."""
    def __init__(self,bundle):
        super().__init__(bundle);self.projection_events=[]

    def to_motor(self,q):
        goal=super().to_motor(q)
        if self.phase>=self.prefix_steps:
            legal=np.clip(goal,np.ceil(self.low).astype(int),np.floor(self.high).astype(int))
            if not np.array_equal(legal,goal):
                self.projection_events.append(dict(phase=self.phase,motor_ids=(np.flatnonzero(legal!=goal)+1).tolist(),
                                                   proposed=goal.tolist(),admissible=legal.tolist()))
            return legal
        return goal
