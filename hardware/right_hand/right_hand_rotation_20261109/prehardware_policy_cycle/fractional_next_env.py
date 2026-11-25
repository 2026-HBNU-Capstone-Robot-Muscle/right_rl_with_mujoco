"""Preserve sub-tick intent while sending at most two integer ticks per sample."""
import numpy as np
import jax
import jax.numpy as jp
from mujoco_playground._src.mjx_env import State
from next_turn_env import NextTurn
from learning_env import decode,SHAPE_OBS,SHAPE_REWARD,SHAPE_METRIC,METRIC_NAMES
from native_cycle import SIM_TO_MOTOR_IDS,motor

SHAPE_FRACTIONAL=jax.ShapeDtypeStruct((210,),np.uint32)
IDS=np.array(SIM_TO_MOTOR_IDS)-1


def accumulate(previous,remainder,action_sim):
    motor_action=np.empty(16);motor_action[IDS]=np.clip(np.asarray(action_sim,float),-1,1)
    continuous=np.asarray(previous,float)+np.asarray(remainder,float)+2*motor_action
    goal=np.asarray(previous,int)+np.clip(np.rint(continuous).astype(int)-np.asarray(previous,int),-2,2)
    remain=np.clip(continuous-goal,-.499999999,.499999999)
    return goal,remain


class FractionalNextTurn(NextTurn):
    def _reset_one(self,rng):
        words,obs=super()._reset_one(rng)
        return np.r_[decode(words),np.zeros(16)].astype(np.float64).view(np.uint32),obs

    def _reset_numpy(self,rng):
        rng=np.asarray(rng);leading=rng.shape[:-1]
        items=[self._reset_one(key) for key in rng.reshape(-1,2)]
        return np.stack([x[0] for x in items]).reshape(leading+(210,)),np.stack([x[1] for x in items]).reshape(leading+(110,))

    def reset(self,rng):
        data,obs=jax.pure_callback(self._reset_numpy,(SHAPE_FRACTIONAL,SHAPE_OBS),rng,vmap_method='broadcast_all')
        z=jp.asarray(0.,jp.float32)
        return State(data=data,obs=obs,reward=z,done=z,metrics={name:z for name in METRIC_NAMES},info={})

    def _step_one(self,words,obs,action):
        raw=decode(words);base=raw[:89].copy().view(np.uint32)
        model,data=self._restore(base);previous=motor(data.ctrl)
        goal,remain=accumulate(previous,raw[89:],action)
        effective=(goal[IDS]-previous[IDS])/2.
        new,new_obs,reward,done,metrics=super()._step_one(base,obs,effective)
        return np.r_[decode(new),remain].astype(np.float64).view(np.uint32),new_obs,reward,done,metrics

    def _step_numpy(self,data,obs,action):
        leading=np.asarray(data).shape[:-1]
        rows=[self._step_one(d,o,a) for d,o,a in zip(np.asarray(data).reshape(-1,210),np.asarray(obs).reshape(-1,110),np.asarray(action).reshape(-1,16))]
        shapes=((210,),(110,),(),(),(len(METRIC_NAMES),))
        return tuple(np.stack([row[i] for row in rows]).reshape(leading+shape) for i,shape in enumerate(shapes))

    def step(self,state,action):
        data,obs,reward,done,values=jax.pure_callback(self._step_numpy,
            (SHAPE_FRACTIONAL,SHAPE_OBS,SHAPE_REWARD,SHAPE_REWARD,SHAPE_METRIC),state.data,state.obs,action,vmap_method='broadcast_all')
        metrics=dict(state.metrics);metrics.update({name:values[i] for i,name in enumerate(METRIC_NAMES)})
        return state.replace(data=data,obs=obs,reward=reward,done=done,metrics=metrics)
