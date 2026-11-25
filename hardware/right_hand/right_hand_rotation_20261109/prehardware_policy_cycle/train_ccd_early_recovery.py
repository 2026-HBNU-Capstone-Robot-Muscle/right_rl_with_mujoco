"""One bounded early-state SAC repair experiment; original checkpoint retained."""
import os
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE','false')
import time
import json
import numpy as np
import jax
from brax.training.agents.sac import train as sac,checkpoint
from train_correction import replicated,wrap
from ccd_early_recovery_env import CCDEarlyRecovery
from learning_env import decode
from native_cycle import OUT,ROOT,save_json,sha


def main():
    folder=OUT/'sac_ccd50_early_recovery_10k'
    if folder.exists():raise RuntimeError('Preserve prior or active early repair')
    env=CCDEarlyRecovery();eval_env=CCDEarlyRecovery(seed=9003,training=False)
    tested=set();probes=[]
    for seed in range(200):
        state,obs=env._reset_one(np.array([seed,0],np.uint32));v=decode(state)
        if int(v[84]) in tested:continue
        tested.add(int(v[84]))
        new,nobs,reward,done,metrics=env._step_one(state,obs,np.zeros(16))
        assert nobs.shape==(110,) and np.isfinite(nobs).all()
        if not metrics[0]:assert reward>=.08
        probes.append(dict(first=int(v[84]),end=int(v[85]),one_step_reward=float(reward)))
        if len(tested)==len(env.starts):break
    assert tested==set(env.starts)
    folder.mkdir()
    restore=OUT/'sac_current_contract_v4_20k/checkpoints/000000010256'
    settings=dict(algorithm='Brax SAC',steps=10000,residual_rad=.01,
        observation_size=110,action_size=16,current_model_sha256=env.provenance['model_sha256'],
        reference_sha256=env.provenance['reference_sha256'],numerical_CCD_iterations=50,
        control_seconds=.075,max_goal_ticks=2,integer_encoder_observations=True,
        restored_actor=str(restore.relative_to(ROOT)),restore_scope='actor and normalization; new critics/optimizers',
        reset_source='actual early CCD50 parent rollout',reset_indices=env.starts.tolist(),horizon=1200,
        reset_probabilities=env.probabilities.tolist(),early_intervention_before_large_late_pose_error=True,
        valid_step_reward_lower_bound=.08,fault_reward=-20.,tests=probes,
        learning_windows_not_continuous_cycle_success=True,planned_deployment_gate=[600,4800],
        native_geometry_limits_force_caps_and_guards_preserved=True,
        devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'settings.json',settings)
    for name in ('ccd_early_recovery_env.py','train_ccd_early_recovery.py','ccd_recovery_env.py','ccd_repair_env.py','learning_env.py','native_cycle.py'):
        (folder/('source_'+name)).write_bytes((OUT/name).read_bytes())
    began=time.perf_counter()
    def progress(steps,metrics):
        row=dict(steps=int(steps),elapsed_seconds=time.perf_counter()-began,
            metrics={k:float(v) for k,v in metrics.items() if getattr(v,'size',1)==1})
        save_json(folder/'progress.json',row)
        with (folder/'metrics.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(row)+'\n')
        print(json.dumps(dict(steps=int(steps),seconds=row['elapsed_seconds'],
            metrics={k:v for k,v in row['metrics'].items() if any(s in k for s in ('fault','reward','window_complete'))})),flush=True)
    jax.device_put_replicated=replicated;old=checkpoint.load
    checkpoint.load=lambda path:replicated(old(path),jax.local_devices()[:1])
    try:
        sac.train(environment=env,eval_env=eval_env,num_timesteps=10000,episode_length=1200,
            num_envs=4,num_eval_envs=4,batch_size=128,min_replay_size=512,max_replay_size=10000,
            grad_updates_per_step=1,num_evals=2,learning_rate=1e-4,discounting=.99,
            normalize_observations=True,deterministic_eval=True,wrap_env_fn=wrap,
            restore_checkpoint_path=str(restore),checkpoint_logdir=str(folder/'checkpoints'),progress_fn=progress,seed=5)
    except Exception as exc:
        save_json(folder/'failure.json',dict(error=repr(exc),devices_opened=0));raise
    finally:checkpoint.load=old
    p=json.loads((folder/'progress.json').read_text())
    save_json(folder/'result.json',dict(settings,status='offline_CCD50_early_recovery_SAC_complete',
        trainer_reported_environment_steps=p['steps'],elapsed_seconds=time.perf_counter()-began,task_success_certified=False))


if __name__=='__main__':main()
