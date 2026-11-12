"""Native MuJoCo SAC residual environment at the actual 2-tick/75ms contract.

The JAX callback wraps unchanged native physics. Episodes sampled from archived
model states are training windows, never evidence of an uninterrupted cycle.
"""
from pathlib import Path
import sys
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))
import importlib.util
import json
import numpy as np
import mujoco
import jax
import jax.numpy as jp
from mujoco_playground._src.mjx_env import State
from scipy.spatial.transform import Rotation, Slerp
from native_cycle import assets, Guard, BOUND, motor, ticks_to_sim, SIM_TO_MOTOR_IDS, TICK
from saved_policy_observation_provider_20261109 import build_observation
from cube_task_frame_20261105 import upright_yaw_target, orientation_error_deg
from upper_layer_cylinder_clearance_20261105 import precompute,gaps

# Keep native double-precision state bit-exact in opaque32-bit words. Globally
# enabling JAX64 changes third-party episode-counter dtypes and breaks scans.
SHAPE_DATA = jax.ShapeDtypeStruct((178,), np.uint32)
SHAPE_OBS = jax.ShapeDtypeStruct((110,), np.float32)
SHAPE_REWARD = jax.ShapeDtypeStruct((), np.float32)
METRIC_NAMES=('fault','pose_error','center_error','action_cost','window_complete','lower_grip','upper_gap','goal_error')
SHAPE_METRIC = jax.ShapeDtypeStruct((len(METRIC_NAMES),), np.float32)


def decode(vector):
    return np.ascontiguousarray(vector,dtype=np.uint32).view(np.float64)


class ResidualCycle:
    observation_size = 110
    action_size = 16
    dt = .075
    backend = 'native_mujoco_callback'

    def __init__(self, residual_rad=.01, horizon=128, seed=0, training=True):
        self.model, self.initial, archive, self.archive_report, self.provenance = assets('active')
        self.reference = archive['goals']
        self.residual_rad = float(residual_rad)
        self.horizon = int(horizon)
        self.training = training
        self.seed = seed
        pool = HERE/'baseline_archived_zero/state_pool.npz'
        report = json.loads((pool.parent/'review.json').read_text())
        if report['stop'] or report['completed_goals'] != len(self.reference):
            raise RuntimeError('A fully replayed nominal model baseline is required')
        self.pool = dict(np.load(pool, allow_pickle=False))
        self.pool_indices = self.pool['indices'].astype(int)
        target = upright_yaw_target(self.initial['initial_cube_rotation'], self.initial['up'], 90)
        self.goal_quat = Rotation.from_matrix(target).as_quat()[[3,0,1,2]]
        self.reference_centers = np.column_stack([np.interp(np.arange(len(self.reference)+1), self.pool_indices,
                                                self.pool['qpos'][:,-7+i]) for i in range(3)])
        rotations = Rotation.from_quat(self.pool['qpos'][:,-4:][:,[1,2,3,0]])
        self.reference_rotations = Slerp(self.pool_indices,rotations)(np.arange(len(self.reference)+1)).as_matrix()
        path = ROOT.parent/'work/Leap_Test/deploy/obs_builder.py'
        spec = importlib.util.spec_from_file_location('cycle_training_observations',path)
        mod = importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
        self.builder_type = mod.ObsBuilder
        self.slices={};cursor=0
        for key in ('qpos','qvel','ctrl','qacc_warmstart'):
            n=getattr(mujoco.MjData(self.model),key).size
            self.slices[key]=slice(cursor,cursor+n);cursor+=n
        self.physics_size=cursor
        if cursor != 83:
            raise RuntimeError('Unexpected model state layout')
        self.lo,self.hi,self.qlo,self.qhi=Guard(self.model).lo,Guard(self.model).hi,Guard(self.model).qlo,Guard(self.model).qhi
        # Current active model plus one deliberately conservative simulated
        # actuator variant. Neither changes a physical 450mA setting.
        self.models=[self.model]
        old,_,_,_,_=assets('archived')
        self.models.append(old)
        self.clearance_cache=[precompute(m,Guard(m).hand) for m in self.models]

    @property
    def unwrapped(self):
        return self

    def _pack(self,data,index,end_index,variant,gap=0.,first=0):
        return np.r_[data.qpos,data.qvel,data.ctrl,data.qacc_warmstart,
                     data.time,index,end_index,variant,gap,first].astype(np.float64).view(np.uint32)

    def _restore(self,vector):
        vector=decode(vector)
        variant=int(vector[86]);model=self.models[variant]
        data=mujoco.MjData(model)
        for key,s in self.slices.items():getattr(data,key)[:]=vector[s]
        data.time=float(vector[83]);mujoco.mj_forward(model,data)
        return model,data

    def _builder(self,obs=None):
        b=self.builder_type()
        if obs is not None:
            b.qpos_err_hist=np.array(obs[16:64],float)
            b.cube_pos_err_hist=np.array(obs[64:73],float)
            b.cube_ori_err_hist=np.array(obs[73:91],float)
            b.last_act=np.array(obs[91:107],float)
        return b

    def observe(self,model,data,index,builder=None):
        b=self._builder() if builder is None else builder
        if builder is None:
            for _ in range(2):build_observation(b,model,data,data.ctrl,self.goal_quat)
        obs=build_observation(b,model,data,data.ctrl,self.goal_quat)
        return np.r_[obs,index/max(1,len(self.reference)-1),float(index>=len(self.reference)-134)].astype(np.float32)

    def _reset_one(self,rng):
        seed=int(np.asarray(rng,dtype=np.uint32).astype(np.uint64).sum())+self.seed
        random=np.random.default_rng(seed)
        variant=int(random.integers(2)) if self.training else 0
        # Hold windows and movement windows share the same observation contract.
        if random.random()<.35:
            pos=int(random.integers(max(0,len(self.pool_indices)-5),len(self.pool_indices)-1))
        else:
            pos=int(random.integers(max(1,len(self.pool_indices)-3)))
        index=int(self.pool_indices[pos]);end=min(index+self.horizon,len(self.reference))
        model=self.models[variant];data=mujoco.MjData(model)
        for key,s in self.slices.items():getattr(data,key)[:]=self.pool[key][pos]
        data.time=float(self.pool['time'][pos]);mujoco.mj_forward(model,data)
        return self._pack(data,index,end,variant,first=index), self.observe(model,data,index)

    def _reset_numpy(self,rng):
        rng=np.asarray(rng);leading=rng.shape[:-1]
        items=[self._reset_one(key) for key in rng.reshape(-1,2)]
        return (np.stack([v[0] for v in items]).reshape(leading+(178,)),
                np.stack([v[1] for v in items]).reshape(leading+(110,)))

    def reset(self,rng):
        data,obs=jax.pure_callback(self._reset_numpy,(SHAPE_DATA,SHAPE_OBS),rng,vmap_method='broadcast_all')
        z=jp.asarray(0.,dtype=jp.float32)
        return State(data=data,obs=obs,reward=z,done=z,metrics={name:z for name in METRIC_NAMES},info={})

    def _step_one(self,vector,obs,action):
        model,data=self._restore(vector)
        vector=decode(vector)
        index=int(vector[84]);end=int(vector[85]);variant=int(vector[86]);first=int(vector[88])
        action=np.clip(np.asarray(action,float),-1,1)
        if index>=len(self.reference):index=len(self.reference)-1
        desired=ticks_to_sim(self.reference[index])+self.residual_rad*action
        previous=motor(data.ctrl);proposal=motor(desired)
        goal=previous+np.clip(proposal-previous,-2,2)
        guard=Guard(model);guard.gap=float(vector[87]);reason=[]
        prior=data.ctrl.copy()
        if np.any(goal<self.lo) or np.any(goal>self.hi):reason=['native_goal_range']
        else:
            data.ctrl[:]=ticks_to_sim(goal)
            for _ in range(75):
                mujoco.mj_step(model,data);reason=guard.check(data)
                if reason:break
        next_index=index+1
        builder=self._builder(obs);builder.set_last_act(np.clip((data.ctrl-prior)/.2,-1,1))
        new_obs=self.observe(model,data,next_index,builder)
        B=data.xmat[model.body('cube').id].reshape(3,3)
        target_B=self.reference_rotations[next_index]
        orientation=orientation_error_deg(B,target_B)
        center=float(np.linalg.norm(data.qpos[-7:-4]-self.reference_centers[next_index])*1000)
        cost=float(np.mean(action**2))
        # Dense tracking of a successful model reference; the external evaluator
        # measures final90, clearance, grip and uninterrupted next rotation.
        reward=1.-.035*orientation-.04*center-.02*cost
        lower_good=0.;upper_gap=0.;goal_error=orientation_error_deg(B,Rotation.from_quat(self.goal_quat[[1,2,3,0]]).as_matrix())
        if next_index>=5636:
            cube=model.geom('cube').id;forces={}
            for ci,c in enumerate(data.contact):
                a,b=int(c.geom1),int(c.geom2)
                if cube not in (a,b):continue
                other=b if a==cube else a;name=model.geom(other).name
                height=float(-(c.pos-data.qpos[-7:-4])@B[:,1]*1000)
                if name in ('if_tip','th_tip','rf_tip') and height<55/6:
                    f=np.zeros(6);mujoco.mj_contactForce(model,data,ci,f)
                    forces[name]=forces.get(name,0.)+float(f[0])
            lower_good=float(all(forces.get(name,0.)>.2 for name in ('if_tip','th_tip')))
            lower_signal=sum(min(forces.get(name,0.)/.2,1.) for name in ('if_tip','th_tip'))
            upper_gap=min(value for _,value in gaps(model,data,data.geom_xpos[cube],
                          data.geom_xmat[cube].reshape(3,3),self.clearance_cache[variant],True))
            reward+=lower_signal+float(np.clip(upper_gap-.3,-5,1))-.1*max(0,goal_error-4)
            reward+=.25*min(forces.get('rf_tip',0.)/.2,1.)
        if reason:reward=-20.
        complete=next_index>=end
        done=bool(reason) or complete
        new=self._pack(data,next_index,end,variant,guard.gap,first)
        metrics=np.array([bool(reason),orientation,center,cost,complete and not reason,lower_good,upper_gap,goal_error],np.float32)
        return new,new_obs,np.float32(reward),np.float32(done),metrics

    def _step_numpy(self,data,obs,action):
        leading=np.asarray(data).shape[:-1]
        items=[self._step_one(d,o,a) for d,o,a in zip(np.asarray(data).reshape(-1,178),
               np.asarray(obs).reshape(-1,110),np.asarray(action).reshape(-1,16))]
        shapes=((178,),(110,),(),(),(len(METRIC_NAMES),))
        return tuple(np.stack([item[i] for item in items]).reshape(leading+shape) for i,shape in enumerate(shapes))

    def step(self,state,action):
        data,obs,reward,done,values=jax.pure_callback(self._step_numpy,
            (SHAPE_DATA,SHAPE_OBS,SHAPE_REWARD,SHAPE_REWARD,SHAPE_METRIC),
            state.data,state.obs,action,vmap_method='broadcast_all')
        metrics=dict(state.metrics)
        metrics.update({name:values[i] for i,name in enumerate(METRIC_NAMES)})
        return state.replace(data=data,obs=obs,reward=reward,done=done,
                             metrics=metrics)
