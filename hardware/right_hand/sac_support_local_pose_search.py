"""Offline x/z placement sweep near the qualitative support photograph."""
import json
import sys
from pathlib import Path

import jax
import mujoco
import numpy as np
from mujoco_playground import registry

from sac_cube_model import set_cube_physics


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from leap_hw import hw_to_sim


def main():
    config = registry.get_default_config('LeapCubeFaceUp')
    config.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=config)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
    trial = json.loads((ROOT / 'leap_support_capture_result.json').read_text(encoding='utf-8'))
    ticks = [r for r in trial['samples'] if r['phase'] == 'place_cube'][-1]['actual']
    q[:16] = hw_to_sim(np.asarray(ticks) * 2 * np.pi / 4096)
    anchor = json.loads((ROOT / 'sac_registration_support_photo_diagnostic.json').read_text(encoding='utf-8'))
    base = np.asarray(anchor['cube_center_in_sim_world_candidate_m'], dtype=float)
    q[-4:] = anchor['cube_quat_world_candidate']
    cube = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    rows = []
    for dx_mm in (-20, -15, -10, -5, 0, 5, 10, 15, 20):
        for dz_mm in (-5, 0, 5, 10, 15, 20):
            point = base + np.array([dx_mm, 0, dz_mm]) / 1000
            q[-7:-4] = point
            data = mujoco.MjData(model)
            data.qpos[:] = q
            data.ctrl[:] = q[:16]
            mujoco.mj_forward(model, data)
            def contacts():
                found = []
                for c in data.contact:
                    if cube in (c.geom1, c.geom2):
                        other = c.geom2 if c.geom1 == cube else c.geom1
                        found.append((float(c.dist), mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other)))
                return found
            initial = contacts()
            for _ in range(30):
                data.qpos[:16] = q[:16]
                data.qvel[:16] = 0
                mujoco.mj_forward(model, data)
                mujoco.mj_step(model, data)
            final = contacts()
            rows.append({'dx_mm': dx_mm, 'dz_mm': dz_mm,
                         'initial_contacts': len(initial),
                         'initial_penetration_mm': -1000 * min([0.] + [v[0] for v in initial]),
                         'final_contacts': len(final),
                         'initial_geoms': sorted({v[1] for v in initial}),
                         'drift_mm': float(1000 * np.linalg.norm(data.qpos[-7:-4] - point))})
    plausible = [r for r in rows if r['initial_contacts'] and r['final_contacts']
                 and r['initial_penetration_mm'] <= 2]
    result = {'photographed_start_world_m': base.tolist(), 'rows': rows,
              'plausible_contact_count': len(plausible),
              'best_plausible_contact': sorted(plausible, key=lambda r: r['drift_mm'])[:10],
              'hardware_authorized': False,
              'caveat': 'Grid and single-marker reset are provisional; this is not a calibrated grasp.'}
    (ROOT / 'sac_support_local_pose_search_result.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'plausible_contact_count': len(plausible),
                      'best_plausible_contact': result['best_plausible_contact'][:5]}, indent=2))


if __name__ == '__main__':
    main()
