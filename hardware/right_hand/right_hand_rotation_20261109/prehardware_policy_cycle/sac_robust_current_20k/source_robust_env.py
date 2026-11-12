"""Explicit simulated placement/friction/gain perturbations for SAC continuation."""
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from learning_env import ResidualCycle,decode
from native_cycle import assets,Guard


class RobustResidualCycle(ResidualCycle):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        from upper_layer_cylinder_clearance_20261105 import precompute
        for friction,gain in ((.25,1.),(.35,1.),(.3,.9),(.3,1.1)):
            model,_,_,_,_=assets('active');guard=Guard(model);cube=model.geom('cube').id
            for gid in range(model.ngeom):
                if int(model.geom_bodyid[gid]) in guard.hand or gid==cube:model.geom_friction[gid,0]=friction
            model.actuator_gainprm[:,0]*=gain;model.actuator_biasprm[:,1:3]*=gain
            self.models.append(model);self.clearance_cache.append(precompute(model,guard.hand))

    def _reset_one(self,rng):
        words,obs=super()._reset_one(rng);v=decode(words).copy()
        random=np.random.default_rng(int(np.asarray(rng,dtype=np.uint32).astype(np.uint64).sum())+self.seed+733)
        variant=int(random.integers(len(self.models))) if self.training else 0
        v[86]=variant;model,data=self._restore(v.view(np.uint32));nominal=data.qpos.copy()
        # Reject geometrically invalid starts rather than enlarging any bound.
        for attempt in range(8):
            data.qpos[:]=nominal
            offset=random.uniform(-.5,.5,size=2)/1000
            yaw=random.uniform(-1.,1.)
            if not self.training:
                offset=np.array([.35,-.35])/1000;yaw=.5
            data.qpos[-7:-5]+=offset
            data.qpos[-4:]=(Rotation.from_rotvec(self.initial['up']*np.radians(yaw))*
                            Rotation.from_quat(nominal[-4:][[1,2,3,0]])).as_quat()[[3,0,1,2]]
            mujoco.mj_forward(model,data)
            if not Guard(model).check(data):break
        else:
            data.qpos[:]=nominal;mujoco.mj_forward(model,data)
        index=int(v[84]);return self._pack(data,index,int(v[85]),variant,first=int(v[88])),self.observe(model,data,index)

    def _step_one(self,words,obs,action):
        new,new_obs,reward,done,metrics=super()._step_one(words,obs,action)
        v=decode(new)
        if v[84]>=5636 and metrics[0]==0:
            gap=float(metrics[6]);missing=max(0.,2.-gap)
            reward=np.float32(float(reward)-.8*missing*missing+float(metrics[5]))
        return new,new_obs,reward,done,metrics
