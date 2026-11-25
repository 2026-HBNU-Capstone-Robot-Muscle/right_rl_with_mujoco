"""Offline fixed-target comparison of teacher and trained residual SAC."""

import argparse
import json
from pathlib import Path

import jax
import jax.numpy as jp
import numpy as np
from brax.training.agents.sac import checkpoint
from mujoco_playground import registry

from sac_real_start_env import RealStartFaceUp
from sac_residual_real_start_env import ResidualFaceUp

ROOT = Path(__file__).resolve().parent


def evaluate(env, actor, seed, max_steps):
    state = jax.jit(env.reset)(jax.random.PRNGKey(seed))
    start = np.asarray(state.data.qpos[-7:-4])
    step_fn = jax.jit(env.step)
    first_success = None
    steps = 0
    for index in range(max_steps):
        if bool(state.done):
            break
        action = jp.zeros(env.action_size) if actor is None else actor(
            state.obs['state'], jax.random.PRNGKey(index))[0]
        state = step_fn(state, action)
        steps = index + 1
        if bool(float(state.metrics.get('reward/success', 0.))) and first_success is None:
            first_success = steps
    return {'seed': seed, 'steps': steps, 'first_success_step': first_success,
            'dropped_or_done': bool(state.done),
            'cube_displacement_mm': float(1000 * np.linalg.norm(
                np.asarray(state.data.qpos[-7:-4]) - start)),
            'final_orientation_error': float(state.metrics.get('orientation_error', 0.))}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', default='sac_real_start_residual_100k')
    parser.add_argument('--faces', nargs='+', type=int, default=[3])
    parser.add_argument('--seeds', nargs='+', type=int, default=[0, 1, 2])
    parser.add_argument('--max-steps', type=int, default=300)
    args = parser.parse_args()
    checkpoints = sorted((ROOT / args.run / 'checkpoints').glob('[0-9]*'))
    if not checkpoints:
        raise RuntimeError('No SAC checkpoint found')
    actor = checkpoint.load_policy(str(checkpoints[-1]), deterministic=True)
    actor = jax.jit(actor)
    rows = []
    for face in args.faces:
        cfg = registry.get_default_config('LeapCubeFaceUp')
        cfg.impl = 'jax'
        cfg.target_face = face
        env = ResidualFaceUp(RealStartFaceUp(registry.load('LeapCubeFaceUp', config=cfg)))
        for seed in args.seeds:
            for mode, policy in (('teacher_only', None), ('teacher_plus_sac', actor)):
                result = evaluate(env, policy, seed, args.max_steps)
                row = {'target_face': face, 'mode': mode, **result}
                rows.append(row)
                print(row, flush=True)
    report = {'status': 'offline_fixed_target_residual_eval',
              'hardware_authorized': False,
              'checkpoint': str(checkpoints[-1]),
              'results': rows,
              'caveat': 'A few deterministic simulated starts do not validate camera occlusion or physical flipping.'}
    (ROOT / args.run / 'target_eval_result.json').write_text(
        json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
