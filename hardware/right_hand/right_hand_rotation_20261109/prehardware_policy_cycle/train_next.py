"""Offline short-turn SAC curriculum from the validated first lower grip."""
import os
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE','false')
import argparse
import time
import json
import numpy as np
import jax
from brax.training.agents.sac import train as sac,checkpoint
from train_correction import replicated,wrap
from next_turn_env import NextTurn
from native_cycle import OUT,ROOT,save_json,sha


def main():
    p=argparse.ArgumentParser();p.add_argument('--turn',type=float,default=5.)
    p.add_argument('--steps',type=int,default=10000);p.add_argument('--name',default='sac_next_turn5_10k')
    p.add_argument('--fractional',action='store_true');a=p.parse_args()
    folder=OUT/a.name
    if folder.exists():raise RuntimeError('Keep prior runs and choose a new name')
    env_type=NextTurn
    if a.fractional:
        from fractional_next_env import FractionalNextTurn,accumulate
        env_type=FractionalNextTurn
        previous=np.full(16,2048);remainder=np.zeros(16)
        for _ in range(100):
            goal,remainder=accumulate(previous,remainder,np.full(16,.1))
            if np.max(abs(goal-previous))>2:raise AssertionError('Fractional control exceeded rate limit')
            previous=goal
        if not np.all(abs(previous-2068)<=1):raise AssertionError('Small controls were lost')
    folder.mkdir();env=env_type(a.turn);eval_env=env_type(a.turn,training=False)
    # Test actual wrapped reset/step scan before starting this task's trainer.
    from brax.envs.wrappers.training import EvalWrapper
    import jax.numpy as jp
    wrapped=EvalWrapper(wrap(env,episode_length=256,action_repeat=1,randomization_fn=None))
    state=jax.jit(wrapped.reset)(jax.random.split(jax.random.PRNGKey(0),4))
    @jax.jit
    def test(state):
        def body(state,_):return wrapped.step(state,jp.zeros((4,16),jp.float32)),None
        return jax.lax.scan(body,state,None,length=8)[0]
    tested=test(state)
    if tested.obs.shape!=(4,110):raise AssertionError('Next-turn observation contract')
    restore=OUT/'sac_current_contract_v4_20k/checkpoints/000000010256'
    config={'algorithm':'SAC','timesteps':a.steps,'curriculum_increment_deg':a.turn,
            'goal_tolerance_deg':env.tolerance,'control_seconds':.075,'max_goal_ticks':2,
            'output_mode':'incremental_joint_ticks_not_reference_residual',
            'reference_trajectory_used_for_actions':False,'restored_actor':str(restore.relative_to(ROOT)),
            'fractional_tick_intent_accumulated':a.fractional,
            'starting_model_grip':'first current-model SAC rotation and hold endpoint',
            'new_goal_is_curriculum_relative_not_final_absolute180_validation':True,
            'requires54_goal_hold_samples_and80percent_lower_tip_presence':True,
            'source_sha256':{'next_turn_env.py':sha(OUT/'next_turn_env.py'),'train_next.py':sha(OUT/'train_next.py')},
            'devices_opened':0,'hardware_execution_allowed':False}
    save_json(folder/'settings.json',config)
    for name in ('next_turn_env.py','train_next.py','learning_env.py','native_cycle.py','fractional_next_env.py'):
        (folder/('source_'+name)).write_bytes((OUT/name).read_bytes())
    jax.device_put_replicated=replicated;load=checkpoint.load
    checkpoint.load=lambda path:replicated(load(path),jax.local_devices()[:1])
    began=time.perf_counter()
    def progress(steps,metrics):
        row={'steps':int(steps),'elapsed_seconds':time.perf_counter()-began,
             'metrics':{key:float(value) for key,value in metrics.items() if getattr(value,'size',1)==1}}
        save_json(folder/'progress.json',row)
        with (folder/'metrics.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(row)+'\n')
        print(json.dumps({'steps':int(steps),'seconds':row['elapsed_seconds'],
                         'metrics':{key:value for key,value in row['metrics'].items() if 'window_complete' in key or 'reward' in key or 'fault' in key}}),flush=True)
    try:
        sac.train(environment=env,eval_env=eval_env,num_timesteps=a.steps,episode_length=256,
                  num_envs=4,num_eval_envs=4,batch_size=128,min_replay_size=512,
                  max_replay_size=max(10000,a.steps),grad_updates_per_step=1,num_evals=3,
                  learning_rate=1e-4,discounting=.99,normalize_observations=True,
                  deterministic_eval=True,wrap_env_fn=wrap,restore_checkpoint_path=str(restore),
                  checkpoint_logdir=str(folder/'checkpoints'),progress_fn=progress,seed=2)
    except Exception as exc:
        save_json(folder/'failure.json',{'error':repr(exc),'devices_opened':0});raise
    finally:checkpoint.load=load
    save_json(folder/'result.json',dict(config,status='offline_curriculum_training_complete',
              elapsed_seconds=time.perf_counter()-began,second90_degree_rotation_certified=False))


if __name__=='__main__':main()
