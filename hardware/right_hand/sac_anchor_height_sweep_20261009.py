"""Offline sensitivity of same-run cube contact to marker-normal depth error."""

import json
import argparse
import sys
from pathlib import Path

import jax
import mujoco
import numpy as np
from mujoco_playground import registry

from hand_registration_geometry import candidate
from sac_cube_model import set_cube_physics
from sac_index_middle_support_probe import probe


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from leap_hw import hw_to_sim


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--registration-file', default='sac_registration_anchor_result.json')
    parser.add_argument('--output-file', default='sac_anchor_height_sweep_20261009_result.json')
    args = parser.parse_args()
    reg = json.loads((ROOT / args.registration_file).read_text(encoding='utf-8'))
    trial = json.loads((ROOT / 'leap_sequence_flip_20261009_221628_602843.json').read_text(encoding='utf-8'))
    ticks = [r for r in trial['samples'] if r['phase'] == 'place_cube'][-1]['actual']
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    cube_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
    q[:16] = hw_to_sim(np.asarray(ticks) * 2 * np.pi / 4096)
    q[-4:] = reg['cube_quat_world_candidate']
    center = np.asarray(reg.get('cube_center_in_sim_world_candidate_m', reg.get('cube_center_world_m')))
    normal = np.asarray(candidate()['hand_to_world_candidate']) @ np.array([0., 0., 1.])
    rows = []
    for offset in (-10, -8, -6, -4, -2, 0, 2):
        q[-7:-4] = center + normal * offset / 1000
        rows.append({'marker_normal_offset_mm': offset,
                     'center_world_m': q[-7:-4].tolist(), **probe(model, q, cube_geom)})
    report = {'status': 'offline_same_run_depth_sensitivity', 'hardware_authorized': False,
              'registration_file': args.registration_file, 'marker_normal_world': normal.tolist(), 'rows': rows,
              'caveat': 'This does not replace measured placement. One marker and marker-to-model transform remain uncertain.'}
    (ROOT / args.output_file).write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
