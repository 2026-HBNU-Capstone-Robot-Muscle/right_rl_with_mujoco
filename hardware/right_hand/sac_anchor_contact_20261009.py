"""Offline contact check of the same-run October 9 stage-0 camera registration."""

import json
import argparse
import sys
from pathlib import Path

import jax
import mujoco
import numpy as np
from mujoco_playground import registry

from sac_cube_model import set_cube_physics
from sac_index_middle_support_probe import probe


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from leap_hw import hw_to_sim


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--registration-file', default='sac_registration_anchor_result.json')
    parser.add_argument('--output-file', default='sac_anchor_contact_20261009_result.json')
    parser.add_argument('--trial-file', default='leap_sequence_flip_20261009_221628_602843.json')
    args = parser.parse_args()
    registration = json.loads((ROOT / args.registration_file).read_text(encoding='utf-8'))
    source = registration.get('source_folder', registration.get('source', ''))
    if 'sequence_vision_' not in source:
        raise RuntimeError('Registration must point to a saved sequence vision capture')
    trial_file = ROOT / args.trial_file
    trial = json.loads(trial_file.read_text(encoding='utf-8'))
    vision_folder = trial.get('vision_recording', {}).get('folder', '')
    if vision_folder and vision_folder.replace('\\', '/').split('/')[-1] not in source:
        raise RuntimeError('Registration and motor samples must be from the same capture')
    samples = [r for r in trial['samples'] if r['phase'] == 'place_cube']
    if not samples:
        raise RuntimeError('No measured place_cube joint sample')
    q = None
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    cube_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
    q[:16] = hw_to_sim(np.asarray(samples[-1]['actual']) * 2 * np.pi / 4096)
    q[-7:-4] = registration.get('cube_center_in_sim_world_candidate_m',
                                registration.get('cube_center_world_m'))
    q[-4:] = registration['cube_quat_world_candidate']
    report = {'status': 'offline_same_run_contact_diagnostic', 'hardware_authorized': False,
              'trial_file': str(trial_file), 'registration_file': args.registration_file,
              'cube_center_world_m': q[-7:-4].tolist(),
              'cube_center_hand_marker_m': registration.get('cube_center_in_hand_marker_m',
                                                        registration.get('cube_center_hand_marker_m')),
              'marker_count': registration.get('cube_marker_count', 1),
              **probe(model, q, cube_geom),
              'caveat': 'Camera pose and marker-to-model transform remain provisional; contact is an offline diagnostic.'}
    (ROOT / args.output_file).write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
