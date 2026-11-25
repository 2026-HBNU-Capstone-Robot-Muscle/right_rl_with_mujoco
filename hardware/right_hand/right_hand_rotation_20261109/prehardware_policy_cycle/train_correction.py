"""Offline SAC continuation of the saved actor on the current model contract."""
import os
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE','false')
import argparse
import hashlib
import json
import time
import jax
import jax.numpy as jp
from brax.training.agents.sac import train as sac
from brax.training.agents.sac import checkpoint
from mujoco_playground import wrapper
from learning_env import ResidualCycle,HERE,ROOT
from native_cycle import save_json,sha


def replicated(value,devices):
    if len(devices)!=1:raise RuntimeError('Validated compatibility path supports one device')
    return jax.tree.map(lambda leaf:jp.expand_dims(jp.asarray(leaf),0),value)


def wrap(env,**kwargs):
    return wrapper.wrap_for_brax_training(env,full_reset=True,**kwargs)


def main():
    p=argparse.ArgumentParser();p.add_argument('--timesteps',type=int,default=20000)
    p.add_argument('--name',default='sac_current_contract_20k');p.add_argument('--seed',type=int,default=0)
    p.add_argument('--residual-rad',type=float,default=.01)
    p.add_argument('--robust',action='store_true')
    p.add_argument('--restore',default='sac_forward_support_reference_v1_20k/checkpoints/000000020000')
    a=p.parse_args()
    verification=json.loads((HERE/'learning_env_verification.json').read_text())
    if verification['status']!='passed':raise RuntimeError('Environment verification required')
    out=HERE/a.name
    if out.exists():raise RuntimeError('Choose a new run name; keep existing checkpoints')
    out.mkdir();jax.device_put_replicated=replicated
    env_type=ResidualCycle
    if a.robust:
        from robust_env import RobustResidualCycle
        env_type=RobustResidualCycle
    env=env_type(residual_rad=a.residual_rad,horizon=128,seed=a.seed)
    eval_env=env_type(residual_rad=a.residual_rad,horizon=128,seed=9001,training=False)
    original=ROOT/a.restore
    if not original.is_dir():raise RuntimeError('Restore checkpoint missing')
    config={'status':'running','algorithm':'Brax SAC','steps':a.timesteps,
            'restored_policy_checkpoint':str(original.relative_to(ROOT)),
            'restore_scope':'actor and running normalization; critic and optimizers initialized by installed trainer',
            'observation_size':110,'action_size':16,'residual_rad':a.residual_rad,
            'control_period_seconds':.075,'maximum_goal_increment_ticks':2,
            'current_model_sha256':env.provenance['model_sha256'],
            'reference_sha256':env.provenance['reference_sha256'],
            'learning_windows_not_full_cycle_trials':True,
            'training_model_variants':['active model','archived model with explicit0.15Nm simulated actuator cap'],
            'learning_env_sha256':sha(HERE/'learning_env.py'),
            'trainer_sha256':sha(HERE/'train_correction.py'),
            'reward_includes_lower_tip_contacts_upper_space_and_target_orientation':True,
            'robust_placement_friction_gain_training':a.robust,
            'robust_clearance_margin_target_mm':2. if a.robust else None,
            'simulation_torque_cap_not_real450mA_conversion':True,
            'devices_opened':0,'hardware_execution_allowed':False,'seed':a.seed}
    save_json(out/'settings.json',config)
    snapshots={}
    for name in ('learning_env.py','native_cycle.py','robust_env.py','train_correction.py'):
        path=HERE/name;snapshot=out/('source_'+name)
        snapshot.write_bytes(path.read_bytes());snapshots[name]=sha(path)
    save_json(out/'source_hashes.json',snapshots)
    began=time.perf_counter()
    def progress(steps,metrics):
        numbers={key:float(value) for key,value in metrics.items() if getattr(value,'size',1)==1}
        with (out/'metrics.jsonl').open('a',encoding='utf-8') as stream:stream.write(json.dumps({'steps':int(steps),'metrics':numbers})+'\n')
        save_json(out/'progress.json',{'steps':int(steps),'elapsed_seconds':time.perf_counter()-began,'metrics':numbers})
        print(json.dumps({'steps':int(steps),'seconds':time.perf_counter()-began,
                          'metrics':{key:value for key,value in numbers.items() if 'fault' in key or 'window_complete' in key or 'reward' in key}}),flush=True)
    original_load=checkpoint.load
    # The installed trainer replaces an already-replicated training state with
    # plain NumPy checkpoint leaves. Restore the same one-device replica layout
    # as its initialized state, without changing the checkpoint or installed code.
    def load_replicated(path):
        return replicated(original_load(path),jax.local_devices()[:1])
    checkpoint.load=load_replicated
    try:
        sac.train(environment=env,eval_env=eval_env,num_timesteps=a.timesteps,
                  episode_length=128,num_envs=4,num_eval_envs=4,batch_size=128,
                  min_replay_size=512,max_replay_size=max(10000,a.timesteps),
                  grad_updates_per_step=1,num_evals=3,learning_rate=1e-4,
                  discounting=.99,normalize_observations=True,deterministic_eval=True,
                  wrap_env_fn=wrap,restore_checkpoint_path=str(original),
                  checkpoint_logdir=str(out/'checkpoints'),progress_fn=progress,seed=a.seed)
    except Exception as exc:
        save_json(out/'failure.json',{'status':'training_failed','error':repr(exc),
                  'elapsed_seconds':time.perf_counter()-began,'devices_opened':0})
        raise
    finally:
        checkpoint.load=original_load
    config.update(status='offline_sac_correction_training_complete',elapsed_seconds=time.perf_counter()-began,
                  task_success_certified=False)
    save_json(out/'result.json',config);print(json.dumps(config),flush=True)


if __name__=='__main__':main()
