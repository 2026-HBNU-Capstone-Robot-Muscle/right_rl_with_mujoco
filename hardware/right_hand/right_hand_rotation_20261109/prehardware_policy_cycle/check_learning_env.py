"""Native/reference parity and batched JAX-callback checks for the SAC bridge."""
import json
import time
import numpy as np
import mujoco
import jax
import jax.numpy as jp
from learning_env import ResidualCycle,HERE,decode
from mujoco_playground import wrapper
from brax.envs.wrappers.training import EvalWrapper
from native_cycle import Guard,ticks_to_sim,motor,save_json


def main():
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--robust',action='store_true');args=parser.parse_args()
    env_type=ResidualCycle
    if args.robust:
        from robust_env import RobustResidualCycle
        env_type=RobustResidualCycle
    began=time.perf_counter();env=env_type(horizon=64)
    pos=int(np.flatnonzero(env.pool_indices==100)[0]);index=100
    parity=[]
    for variant in (0,1):
        model=env.models[variant];data=mujoco.MjData(model)
        for name in env.slices:getattr(data,name)[:]=env.pool[name][pos]
        data.time=env.pool['time'][pos];mujoco.mj_forward(model,data)
        vector=env._pack(data,index,index+64,variant,first=index)
        obs=env.observe(model,data,index)
        # Independent native integration of the unchanged zero-correction goal.
        data.ctrl[:]=ticks_to_sim(env.reference[index]);guard=Guard(model)
        for _ in range(75):
            mujoco.mj_step(model,data)
            if guard.check(data):raise AssertionError('Chosen reference parity state is not admissible')
        expected=data.qpos.copy()
        state=env.reset(jax.random.PRNGKey(0)).replace(data=jp.asarray(vector),obs=jp.asarray(obs))
        updated=jax.jit(env.step)(state,jp.zeros(16,dtype=jp.float32))
        actual=decode(np.asarray(updated.data))
        error=float(np.max(np.abs(expected-actual[env.slices['qpos']])))
        parity.append({'variant':variant,'maximum_joint_and_cube_state_error':error})
        if error>1e-9:raise AssertionError(parity)
        for sign in (-1.,1.):
            result=jax.jit(env.step)(state,jp.full(16,sign,dtype=jp.float32))
            maximum=int(np.max(np.abs(motor(decode(np.asarray(result.data))[env.slices['ctrl']])-motor(decode(vector)[env.slices['ctrl']]))))
            if maximum>2:raise AssertionError('Residual bypassed rate limit')
    reset=jax.jit(jax.vmap(env.reset));step=jax.jit(jax.vmap(env.step))
    state=reset(jax.random.split(jax.random.PRNGKey(123),4))
    for _ in range(4):state=step(state,jp.zeros((4,16),dtype=jp.float32))
    if np.asarray(state.obs).shape!=(4,110) or not np.isfinite(np.asarray(state.data).copy().view(np.float64)).all():raise AssertionError('Batched callback state invalid')
    wrapped=EvalWrapper(wrapper.wrap_for_brax_training(env,episode_length=64,action_repeat=1,full_reset=True))
    state=jax.jit(wrapped.reset)(jax.random.split(jax.random.PRNGKey(321),4))
    @jax.jit
    def unroll(state):
        def step(state,_):return wrapped.step(state,jp.zeros((4,16),dtype=jp.float32)),None
        return jax.lax.scan(step,state,None,length=16)[0]
    state=unroll(state)
    result={'status':'passed','native_75ms_zero_action_parity':parity,
            'environment_class':type(env).__name__,
            'extreme_actions_preserve_max_two_tick_goal_increment':True,
            'batched_callback_envs':4,'batched_steps':4,
            'state_storage':'opaque uint32 words preserving native float64 bits',
            'full_training_and_evaluation_wrappers_scanned_steps':16,
            'control_period_seconds':env.dt,'residual_max_rad':env.residual_rad,
            'training_resets_are_model_windows_not_continuous_success':True,
            'elapsed_seconds':time.perf_counter()-began,'devices_opened':0,'hardware_execution_allowed':False}
    save_json(HERE/'learning_env_verification.json',result);print(json.dumps(result),flush=True)


if __name__=='__main__':main()
