"""Continue an existing SAC actor on converged CCD physics; local model only."""
import os
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE', 'false')
import argparse
import time
import json
from pathlib import Path
import numpy as np
import jax
import jax.numpy as jp
from brax.training.agents.sac import train as sac, checkpoint
from train_correction import replicated, wrap
from ccd_repair_env import CCDRepair
from learning_env import decode
from native_cycle import OUT, ROOT, save_json, sha, motor


def main():
    p=argparse.ArgumentParser(); p.add_argument('--steps', type=int, default=10000)
    p.add_argument('--name', default='sac_ccd50_repair_10k'); a=p.parse_args()
    folder=OUT/a.name
    if folder.exists(): raise RuntimeError('Keep prior runs; choose a new name')
    env=CCDRepair(); eval_env=CCDRepair(seed=9001, training=False)
    checks=[]
    for seed in range(3):
        vector, obs=env._reset_one(np.asarray([seed, 0], np.uint32))
        m,d=env._restore(vector)
        decoded=decode(vector)
        roundtrip=env._pack(d,int(decoded[84]),int(decoded[85]),0,first=int(decoded[88]))
        difference=float(np.max(abs(decode(roundtrip)-decoded)))
        if difference>1e-12:raise AssertionError('Native packed state changed during restore')
        checks.append(dict(start=int(decoded[84]),packed_roundtrip_max_abs=difference))
        before=motor(d.ctrl)
        new,nobs,reward,done,metrics=env._step_one(vector,obs,np.zeros(16))
        _,after=env._restore(new)
        if np.max(abs(motor(after.ctrl)-before))>2 or nobs.shape!=(110,) or not np.isfinite(nobs).all():
            raise AssertionError('Repair contract failed')
        checks[-1].update(one_step_goal_delta_ticks=int(np.max(abs(motor(after.ctrl)-before))), reward=float(reward))
        if m.opt.ccd_iterations!=50 or not np.allclose(obs[:16]/(2*np.pi/4096)+2048,np.rint(obs[:16]/(2*np.pi/4096)+2048),atol=.0002):
            raise AssertionError('CCD/encoder training contract')
    from brax.envs.wrappers.training import EvalWrapper
    wrapped=EvalWrapper(wrap(env, episode_length=512, action_repeat=1, randomization_fn=None))
    state=jax.jit(wrapped.reset)(jax.random.split(jax.random.PRNGKey(0),4))
    @jax.jit
    def test(state):
        def body(s,_):return wrapped.step(s,jp.zeros((4,16),jp.float32)),None
        return jax.lax.scan(body,state,None,length=8)[0]
    tested=test(state)
    if tested.obs.shape!=(4,110): raise AssertionError('Wrapped observation layout')
    folder.mkdir()
    restore=OUT/'sac_current_contract_v4_20k/checkpoints/000000010256'
    settings=dict(algorithm='Brax SAC',steps=a.steps,residual_rad=.01,
        observation_size=110,action_size=16,integer_encoder_observations=True,
        current_model_sha256=env.provenance['model_sha256'], reference_sha256=env.provenance['reference_sha256'],
        numerical_CCD_iterations=50,control_seconds=.075,max_goal_ticks=2,
        restored_actor=str(restore.relative_to(ROOT)),restore_scope='actor and normalization; new critics/optimizers',
        reset_indices=[3000,3400,3600],reset_source='actual free-physics CCD50 parent rollout',
        learning_windows_not_continuous_cycle_success=True,planned_deployment_gate=[3000,4300],
        native_geometry_limits_force_caps_and_guards_preserved=True,contract_checks=checks,
        devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'settings.json',settings)
    for name in ('ccd_repair_env.py','train_ccd_repair.py','learning_env.py','native_cycle.py'):
        (folder/('source_'+name)).write_bytes((OUT/name).read_bytes())
    began=time.perf_counter()
    def progress(steps,metrics):
        row=dict(steps=int(steps),elapsed_seconds=time.perf_counter()-began,
                 metrics={k:float(v) for k,v in metrics.items() if getattr(v,'size',1)==1})
        save_json(folder/'progress.json',row)
        with (folder/'metrics.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(row)+'\n')
        print(json.dumps(dict(steps=int(steps),seconds=row['elapsed_seconds'],
            metrics={k:v for k,v in row['metrics'].items() if any(s in k for s in ('window_complete','reward','fault'))})),flush=True)
    jax.device_put_replicated=replicated; old=checkpoint.load
    checkpoint.load=lambda path:replicated(old(path),jax.local_devices()[:1])
    try:
        sac.train(environment=env,eval_env=eval_env,num_timesteps=a.steps,episode_length=512,
            num_envs=4,num_eval_envs=4,batch_size=128,min_replay_size=512,
            max_replay_size=max(10000,a.steps),grad_updates_per_step=1,num_evals=3,
            learning_rate=1e-4,discounting=.99,normalize_observations=True,
            deterministic_eval=True,wrap_env_fn=wrap,restore_checkpoint_path=str(restore),
            checkpoint_logdir=str(folder/'checkpoints'),progress_fn=progress,seed=3)
    except Exception as exc:
        save_json(folder/'failure.json',dict(error=repr(exc),devices_opened=0));raise
    finally:checkpoint.load=old
    save_json(folder/'result.json',dict(settings,status='offline_CCD50_SAC_repair_training_complete',
        elapsed_seconds=time.perf_counter()-began,task_success_certified=False))


if __name__=='__main__':main()
