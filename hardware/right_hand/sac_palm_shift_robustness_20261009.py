"""Offline ±2 mm start uncertainty test of hypothetical lowered palm boxes."""

import json
import sys
from pathlib import Path

import jax
import mujoco
import numpy as np
from mujoco_playground import registry

from hand_registration_geometry import candidate
from sac_cube_model import set_cube_physics
from sac_index_middle_support_probe import contact_geoms, palm_support


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from leap_hw import hw_to_sim


def main():
    reg = json.loads((ROOT / 'sac_registration_anchor_result.json').read_text(encoding='utf-8'))
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
    nominal = np.asarray(reg['cube_center_in_sim_world_candidate_m'])
    normal = np.asarray(candidate()['hand_to_world_candidate']) @ [0., 0., 1.]
    data = mujoco.MjData(model)
    data.qpos[:] = q
    mujoco.mj_forward(model, data)
    geoms = [g for g in range(model.ngeom)
             if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or '').startswith('palm_collision_')]
    for g in geoms:
        body = model.geom_bodyid[g]
        model.geom_pos[g] -= (data.xmat[body].reshape(3, 3).T @ normal) * .008
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
                        timeline.append({'step': step + 1,
                                         'drift_mm': float(1000 * np.linalg.norm(data.qpos[-7:-4] - q[-7:-4])),
                                         'palm_contact': palm_support([name for _, name in found])})
                rows.append({'perturbation_mm': [dx, dy, dz],
                             'initial_penetration_mm': float(-1000 * min([0.] + [d for d, _ in initial])),
                             'initial_palm_contact': palm_support([name for _, name in initial]),
                             'timeline': timeline})
    stable = [r for r in rows if r['initial_penetration_mm'] <= 2 and r['initial_palm_contact']
              and all(t['palm_contact'] and t['drift_mm'] <= 10 for t in r['timeline'])]
    report = {'status': 'offline_hypothetical_palm_robustness', 'hardware_authorized': False,
              'palm_box_shift_mm': 8, 'perturbations_mm': [-2, 0, 2], 'samples': len(rows),
              'stable_count': len(stable),
              'nominal': next(r for r in rows if r['perturbation_mm'] == [0, 0, 0]),
              'best': sorted(rows, key=lambda r: r['timeline'][-1]['drift_mm'])[:5],
              'caveat': 'Palm shift is an unverified hypothesis; physical support was not measured at this precision.'}
    (ROOT / 'sac_palm_shift_robustness_20261009_result.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
