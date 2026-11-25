"""Offline rollout of existing policy in a separately recompiled palm-mesh model."""
import argparse
import json
import sys
from pathlib import Path

import jax
import jax.numpy as jnp
import mujoco
import numpy as np
from mujoco import mjx
from mujoco_playground import registry

from sac_palm_recompiled_collision_probe import make_model


ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent / 'work' / 'Leap_Test'
sys.path.insert(0, str(PROJECT / 'deploy'))
from leap_hw import hw_to_sim
from policy_np import Policy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--registration-file', default='sac_registration_support_photo_diagnostic.json')
    parser.add_argument('--offset-mm', nargs=3, type=float, default=[14, 0, -2])
    parser.add_argument('--target-face', type=int, default=3)
    parser.add_argument('--output-file', default='sac_recompiled_policy_probe_result.json')
    args = parser.parse_args()
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    cfg.target_face = args.target_face
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model, _ = make_model(env, 'visual_without_boxes')
    env._mj_model = model
    env._mjx_model = mjx.put_model(model, impl='jax')
    env._cube_mass = model.body_subtreemass[model.body('cube').id]
    state = env.reset(jax.random.PRNGKey(0))
    trial = json.loads((ROOT / 'leap_support_capture_result.json').read_text(encoding='utf-8'))
    ticks = [r for r in trial['samples'] if r['phase'] == 'place_cube'][-1]['actual']
    joint = hw_to_sim(np.asarray(ticks) * 2 * np.pi / 4096)
    anchor = json.loads((ROOT / args.registration_file).read_text(encoding='utf-8'))
    pos = np.asarray(anchor['cube_center_in_sim_world_candidate_m']) + np.asarray(args.offset_mm) / 1000
    quat = np.asarray(anchor['cube_quat_world_candidate'])
    qpos = state.data.qpos.at[:16].set(jnp.asarray(joint, jnp.float32))
    qpos = qpos.at[-7:-4].set(jnp.asarray(pos, jnp.float32))
    qpos = qpos.at[-4:].set(jnp.asarray(quat, jnp.float32))
    data = state.data.replace(qpos=qpos, qvel=jnp.zeros_like(state.data.qvel),
                              ctrl=jnp.asarray(joint, jnp.float32))
    data = mjx.forward(env.mjx_model, data)
    info = dict(state.info)
    info['motor_targets'] = data.ctrl
    info['start_cube_pos'] = data.qpos[-7:-4]
    for key in ('qpos_error_history', 'cube_pos_error_history', 'cube_ori_error_history'):
        info[key] = jnp.zeros_like(info[key])
    obs = env._get_obs(data, info)
    state = state.replace(data=data, obs=obs, info=info)
    policy = Policy(PROJECT / 'policy' / 'policy_np.npz')
    step = jax.jit(env.step)
    samples = []
    ever_success = False
    for index in range(300):
        if bool(state.done):
            break
        action = jnp.asarray(policy(np.asarray(state.obs['state'])), dtype=jnp.float32)
        state = step(state, action)
        success = bool(float(state.metrics.get('reward/success', 0.0)))
        ever_success |= success
        if index in (0, 9, 19, 39, 59, 79, 99, 149, 199, 249, 299) or bool(state.done):
            samples.append({'step': index+1, 'cube_position_m': np.asarray(state.data.qpos[-7:-4]).tolist(),
                            'cube_quaternion': np.asarray(state.data.qpos[-4:]).tolist(),
                            'success': success, 'done': bool(state.done),
                            'orientation_error': float(state.metrics.get('orientation_error', 0.0))})
    result = {'model': 'recompiled_palm_visual_without_boxes', 'cube_side_m': .0545,
              'cube_mass_kg': .100, 'start_offset_from_single_marker_m':
              (np.asarray(args.offset_mm) / 1000).tolist(),
              'target_face': args.target_face, 'steps_run': index+1, 'ever_success': ever_success,
              'samples': samples, 'hardware_authorized': False,
              'caveat': 'Single candidate/target and unvalidated real-to-sim reset. No hardware inference.'}
    (ROOT / args.output_file).write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'steps_run': index+1, 'ever_success': ever_success,
                      'last': samples[-1]}, indent=2))


if __name__ == '__main__':
    main()
