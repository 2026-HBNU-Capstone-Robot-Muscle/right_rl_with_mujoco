"""Offline local refinement of a palm-supported cube reset; never opens devices."""

import json
import sys
from pathlib import Path

import jax
import mujoco
import numpy as np
from mujoco_playground import registry

from sac_cube_model import set_cube_physics
from sac_index_middle_support_probe import palm_support, probe


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from leap_hw import hw_to_sim


def main():
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    cube_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    trial = json.loads((ROOT / 'leap_sequence_flip_20261008_191152_267924.json').read_text(encoding='utf-8'))
    ticks = [row for row in trial['samples'] if row['phase'] == 'place_cube'][-1]['actual']
    q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
    q[:16] = hw_to_sim(np.asarray(ticks) * 2 * np.pi / 4096)
    anchor = json.loads((ROOT / 'sac_registration_anchor_result.json').read_text(encoding='utf-8'))
    center = np.asarray(anchor['cube_center_in_sim_world_candidate_m'])
    q[-4:] = anchor['cube_quat_world_candidate']
    rows = []
    for dx in range(4, 17, 2):
        for dy in range(-6, 7, 2):
            for dz in range(-6, 7, 2):
                q[-7:-4] = center + np.array([dx, dy, dz]) / 1000
                result = probe(model, q, cube_geom)
                if palm_support(result['initial_geoms']) and palm_support(result['final_geoms']):
                    rows.append({'offset_mm': [dx, dy, dz],
                                 'center_m': q[-7:-4].tolist(), **result})
    feasible = [row for row in rows if row['penetration_mm'] <= 2 and row['drift_mm'] <= 10]
    report = {
        'status': 'offline_palm_contact_refinement',
        'hardware_authorized': False,
        'cube_mass_kg': .100,
        'positions_tested': 343,
        'sustained_palm_contact_count': len(rows),
        'plausible_count': len(feasible),
        'best_palm_contact': sorted(rows, key=lambda r: (
            r['penetration_mm'] > 2, r['drift_mm']))[:20],
        'best_plausible': sorted(feasible, key=lambda r: r['drift_mm'])[:20],
        'caveat': 'Fine scan is centered on an older, unverified one-marker pose. A numerical fit is not a measured physical reset.',
    }
    (ROOT / 'sac_palm_support_refine_result.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k not in
                      ('best_palm_contact', 'best_plausible')}, indent=2))
    print('best_palm_contact', report['best_palm_contact'][:3])
    print('best_plausible', report['best_plausible'][:3])


if __name__ == '__main__':
    main()
