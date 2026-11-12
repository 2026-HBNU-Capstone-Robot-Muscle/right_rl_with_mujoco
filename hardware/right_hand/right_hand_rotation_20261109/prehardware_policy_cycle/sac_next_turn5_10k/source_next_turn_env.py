"""SAC curriculum for changing orientation from a lower grip, without replay.

This uses bounded joint increments rather than a reference trajectory. A5deg
curriculum result is not a second90deg rotation or a complete two-turn cycle.
"""
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from learning_env import ResidualCycle,decode,HERE
from native_cycle import Guard,motor,ticks_to_sim
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from saved_policy_observation_provider_20261109 import build_observation
from upper_layer_cylinder_clearance_20261105 import gaps


class NextTurn(ResidualCycle):
    def __init__(self,increment_deg=5.,horizon=256,seed=0,training=True,**kwargs):
        super().__init__(horizon=horizon,seed=seed,training=training)
        self.increment_deg=float(increment_deg)
        parent=HERE/'sac_current_contract_v4_20k/export_000000010256/evaluation_active_certified'
        import json
        report=json.loads((parent/'review.json').read_text())
        if not report['rotation_and_hold_passed']:raise RuntimeError('Validated first model grip required')
        self.source=dict(np.load(parent/'end_state.npz',allow_pickle=False))
        d=mujoco.MjData(self.model)
        for key in self.slices:getattr(d,key)[:]=self.source[key]
        d.time=float(self.source['time']);mujoco.mj_forward(self.model,d)
        self.start_center=d.qpos[-7:-4].copy()
        self.target=upright_yaw_target(d.xmat[self.model.body('cube').id].reshape(3,3),self.initial['up'],self.increment_deg)
        self.goal_quat=Rotation.from_matrix(self.target).as_quat()[[3,0,1,2]]
        self.tolerance=1.5 if abs(self.increment_deg)<=10 else (3. if abs(self.increment_deg)<=30 else 5.)

    def observe_next(self,model,data,held=0,builder=None):
        b=self._builder() if builder is None else builder
        if builder is None:
            for _ in range(2):build_observation(b,model,data,data.ctrl,self.goal_quat)
        obs=build_observation(b,model,data,data.ctrl,self.goal_quat)
        return np.r_[obs,1.,min(held/54.,1.)].astype(np.float32)

    def _reset_one(self,rng):
        d=mujoco.MjData(self.model)
        for key in self.slices:getattr(d,key)[:]=self.source[key]
        d.time=float(self.source['time']);mujoco.mj_forward(self.model,d)
        return self._pack(d,0,0,0,first=0),self.observe_next(self.model,d)

    def _step_one(self,words,obs,action):
        model,d=self._restore(words);v=decode(words)
        step=int(v[84]);held=int(v[85]);hits=int(v[88])
        B=d.xmat[model.body('cube').id].reshape(3,3)
        before_error=orientation_error_deg(B,self.target)
        action=np.clip(np.asarray(action,float),-1,1);before=d.ctrl.copy();previous=motor(before)
        desired=ticks_to_sim(previous)+2*(2*np.pi/4096)*action
        goal=motor(desired);guard=Guard(model);guard.gap=float(v[87]);reason=[]
        if np.max(abs(goal-previous))>2:raise AssertionError('Next-turn action exceeded two ticks')
        if np.any(goal<guard.lo) or np.any(goal>guard.hi):reason=['native_goal_range']
        else:
            d.ctrl[:]=ticks_to_sim(goal)
            for _ in range(75):
                mujoco.mj_step(model,d);reason=guard.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-self.start_center)>.08:reason=list(reason)+['cube_translation']
                if reason:break
        B=d.geom_xmat[model.geom('cube').id].reshape(3,3);C=d.qpos[-7:-4]
        error=orientation_error_deg(B,self.target);center=float(np.linalg.norm(C-self.start_center)*1000)
        tilt=face_up_error_deg(B,5,self.initial['up'])
        cube=model.geom('cube').id;force={};normals={}
        for ci,c in enumerate(d.contact):
            a,b=int(c.geom1),int(c.geom2)
            if cube not in (a,b):continue
            other=b if a==cube else a;name=model.geom(other).name
            if name in ('if_tip','mf_tip','rf_tip','th_tip') and -(c.pos-C)@B[:,1]*1000<55/6:
                f=np.zeros(6);mujoco.mj_contactForce(model,d,ci,f)
                force[name]=force.get(name,0.)+float(f[0]);normals[name]=c.frame[:3]*(1 if a==cube else -1)
        other_good=[name for name in ('if_tip','mf_tip','rf_tip') if force.get(name,0.)>.2]
        lower=force.get('th_tip',0.)>.2 and bool(other_good)
        gap=min(value for _,value in gaps(model,d,d.geom_xpos[cube],B,self.clearance_cache[0],True))
        pose_good=error<self.tolerance and tilt<5 and center<2 and gap>.3
        held=held+1 if pose_good else 0;hits=hits+int(lower) if pose_good else 0
        success=held>=54 and hits/max(1,held)>.8
        reward=5*(before_error-error)-.02*error-.03*center-.02*float(np.mean(action**2))
        reward+=.05*min(force.get('th_tip',0.)/.2,1.)
        if pose_good:reward+=1.
        if success:reward+=30.
        if reason:reward=-30.
        done=bool(reason) or success or step+1>=self.horizon
        builder=self._builder(obs);builder.set_last_act(np.clip((d.ctrl-before)/.2,-1,1))
        observed=self.observe_next(model,d,held,builder)
        packed=self._pack(d,step+1,held,0,guard.gap,hits)
        metrics=np.array([bool(reason),error,center,np.mean(action**2),success,lower,gap,error],np.float32)
        return packed,observed,np.float32(reward),np.float32(done),metrics
