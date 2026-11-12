"""Offline SAC policy sensitivity to losing the visual cube pose.

The baseline is the simulator's own reset, not the physical stage-0 placement.
Pose-related actor inputs are frozen during a synthetic camera blackout while
joint and action history continue updating.  This is a diagnostic only.
"""

import json
from pathlib import Path

import jax
import jax.numpy as jp
import numpy as np
from mujoco_playground import registry

from sac_residual_real_start_env import JaxTeacher


ROOT = Path(__file__).resolve().parent
VISUAL_HISTORY = slice(64, 91)
VISUAL_SCALAR = 107


def run(env, policy, seed, blackout, max_steps=300):
    reset = jax.jit(env.reset)
    step = jax.jit(env.step)
    state = reset(jax.random.PRNGKey(seed))
    frozen = np.asarray(state.obs['state']).copy()
    start_cube = np.asarray(state.data.qpos[-7:-4])
    first_success = None
    steps = 0
    for index in range(max_steps):
        if bool(state.done):
            break
        obs = np.asarray(state.obs['state']).copy()
        if blackout is not None and blackout[0] <= index < blackout[1]:
            obs[VISUAL_HISTORY] = frozen[VISUAL_HISTORY]
            obs[VISUAL_SCALAR] = frozen[VISUAL_SCALAR]
        else:
            frozen = obs.copy()
        action = policy(jp.asarray(obs))
        state = step(state, action)
        steps = index + 1
        if first_success is None and float(state.metrics.get('reward/success', 0.)) > 0:
            first_success = steps
    return dict(seed=seed, blackout_steps=blackout, steps=steps,
                first_success_step=first_success, done=bool(state.done),
                final_orientation_error=float(state.metrics.get('orientation_error', 0.)),
                final_cube_displacement_mm=float(1000 * np.linalg.norm(
                    np.asarray(state.data.qpos[-7:-4]) - start_cube)))


def main():
    config = registry.get_default_config('LeapCubeFaceUp')
    config.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=config)
    policy = JaxTeacher()
    rows = []
    for seed in (0,):
        for label, interval in (('clear', None), ('blackout_1s', (20, 40)),
                                ('blackout_5s', (20, 120)), ('blackout_10s', (20, 220))):
            row = dict(mode=label, **run(env, policy, seed, interval))
            rows.append(row)
            print(row, flush=True)
    report = dict(status='offline_pose_dropout_probe', motor_commands_sent=0,
                  observation_layout=dict(joints='0:16', joint_history='16:64',
                                          cube_position_orientation_history='64:91',
                                          last_action='91:107', orientation_error='107'),
                  results=rows,
                  caveat='Single simulated seed and default reset. Synthetic frozen pose does not model camera noise, occlusion geometry, or real contact.')
    (ROOT / 'sac_pose_dropout_probe_result.json').write_text(
        json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
