"""Small offline Brax SAC integration pilot for the measured LEAP start.

No motor or camera imports. It is a trainer smoke test, not a usable policy.
"""

import json
import os
from pathlib import Path
import time

os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE', 'false')

import jax
import jax.numpy as jp
from brax.training.agents.sac import train as sac
from mujoco_playground import registry, wrapper

from sac_real_start_env import RealStartFaceUp

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'sac_real_start_pilot'

# The installed Brax release still calls this JAX API removed in the current
# JAX release. The pilot uses one device, so a leading replica axis is enough.
def _one_device_replicated(value, devices):
    if len(devices) != 1:
        raise RuntimeError('This compatibility shim supports exactly one device')
    return jax.tree.map(lambda leaf: jp.expand_dims(jp.asarray(leaf), 0), value)


jax.device_put_replicated = _one_device_replicated


class StateObservation(wrapper.Wrapper):
    _FULL = 'StateObservation_full_obs'

    def _pack(self, state):
        if isinstance(state.obs, dict):
            state.info[self._FULL] = state.obs
            return state.replace(obs=state.obs['state'])
        return state

    def reset(self, rng):
        return self._pack(self.env.reset(rng))

    def step(self, state, action):
        inner = state.replace(obs=state.info[self._FULL]) if self._FULL in state.info else state
        return self._pack(self.env.step(inner, action))

    @property
    def observation_size(self):
        return self.env.observation_size['state'][0]


def wrap_env(env, **kwargs):
    return wrapper.wrap_for_brax_training(StateObservation(env),
                                          full_reset=True, **kwargs)


def progress(steps, metrics):
    fields = {}
    for key in ('eval/episode_reward', 'training/actor_loss'):
        if key in metrics:
            fields[key] = float(metrics[key])
    print('pilot_progress', steps, fields, flush=True)


def main():
    OUT.mkdir(exist_ok=True)
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = RealStartFaceUp(registry.load('LeapCubeFaceUp', config=cfg))
    eval_env = RealStartFaceUp(registry.load('LeapCubeFaceUp', config=cfg))
    started = time.monotonic()
    make_inference_fn, params, _ = sac.train(
        environment=env,
        eval_env=eval_env,
        num_timesteps=2048,
        episode_length=80,
        num_envs=8,
        num_eval_envs=4,
        batch_size=32,
        min_replay_size=64,
        max_replay_size=4096,
        grad_updates_per_step=1,
        num_evals=2,
        learning_rate=3e-5,
        discounting=.99,
        normalize_observations=True,
        wrap_env_fn=wrap_env,
        checkpoint_logdir=str(OUT / 'checkpoints'),
        progress_fn=progress,
        seed=0,
    )
    report = {'status': 'offline_sac_trainer_pilot_complete',
              'hardware_authorized': False, 'num_timesteps': 2048,
              'elapsed_seconds': time.monotonic() - started,
              'checkpoint_dir': str(OUT / 'checkpoints'),
              'caveat': 'A 2048-step integration pilot cannot train a cube-turning policy.'}
    (OUT / 'pilot_result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
