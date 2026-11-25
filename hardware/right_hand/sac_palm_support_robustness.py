"""Offline perturbation test of the fitted palm-contact start."""

import json
import argparse
import sys
from pathlib import Path

import jax
import mujoco
import numpy as np
from mujoco_playground import registry

from sac_cube_model import set_cube_physics
from sac_index_middle_support_probe import contact_geoms, palm_support
from sac_palm_recompiled_collision_probe import make_model


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from leap_hw import hw_to_sim


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model-setting', choices=('original', 'visual_without_boxes'),
                        default='original')
    parser.add_argument('--output-file', default='sac_palm_support_robustness_result.json')
    args = parser.parse_args()
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    if args.model_setting == 'original':
        model = env.mj_model
        set_cube_physics(model, mass_kg=.100)
    else:
        model, _ = make_model(env, args.model_setting)
    cube_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    trial = json.loads((ROOT / 'leap_sequence_flip_20261008_191152_267924.json').read_text(encoding='utf-8'))
    ticks = [row for row in trial['samples'] if row['phase'] == 'place_cube'][-1]['actual']
    q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
    q[:16] = hw_to_sim(np.asarray(ticks) * 2 * np.pi / 4096)
    anchor = json.loads((ROOT / 'sac_registration_anchor_result.json').read_text(encoding='utf-8'))
    q[-4:] = anchor['cube_quat_world_candidate']
    nominal = np.asarray(anchor['cube_center_in_sim_world_candidate_m']) + np.array([16, -2, -2]) / 1000
    rows = []
    for dx in (-2, 0, 2):
        for dy in (-2, 0, 2):
            for dz in (-2, 0, 2):
                q[-7:-4] = nominal + np.array([dx, dy, dz]) / 1000
                data = mujoco.MjData(model)
                data.qpos[:] = q
                data.ctrl[:] = q[:16]
                mujoco.mj_forward(model, data)
                initial = contact_geoms(model, data, cube_geom)
                timeline = []
                for step in range(100):
                    data.qpos[:16] = q[:16]
                    data.qvel[:16] = 0
                    mujoco.mj_forward(model, data)
                    mujoco.mj_step(model, data)
                    if step in (29, 99):
                        found = contact_geoms(model, data, cube_geom)
                        timeline.append({
                            'step': step + 1,
                            'drift_mm': float(1000 * np.linalg.norm(data.qpos[-7:-4] - q[-7:-4])),
                            'palm_contact': palm_support([name for _, name in found]),
                        })
                rows.append({
                    'perturbation_mm': [dx, dy, dz],
                    'initial_penetration_mm': float(-1000 * min([0.] + [d for d, _ in initial])),
                    'initial_palm_contact': palm_support([name for _, name in initial]),
                    'timeline': timeline,
                })
    stable = [row for row in rows if row['initial_penetration_mm'] <= 2
              and row['initial_palm_contact'] and all(
                  t['palm_contact'] and t['drift_mm'] <= 10 for t in row['timeline'])]
    report = {
        'status': 'offline_robustness_diagnostic', 'hardware_authorized': False,
        'model_setting': args.model_setting,
        'nominal_center_m': nominal.tolist(),
        'perturbation_grid_mm': [-2, 0, 2],
        'samples': len(rows), 'stable_count': len(stable),
        'nominal': next(row for row in rows if row['perturbation_mm'] == [0, 0, 0]),
        'best': sorted(rows, key=lambda row: row['timeline'][-1]['drift_mm'])[:8],
        'caveat': 'Numerical fitted start from one-marker image; stability under small offsets does not validate real contact.',
    }
    (ROOT / args.output_file).write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k != 'best'}, indent=2))


if __name__ == '__main__':
    main()
