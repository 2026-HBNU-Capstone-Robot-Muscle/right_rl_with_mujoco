"""Motor-free local cube-position sensitivity with measured tabletop gravity."""

import json
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
    tilt = json.loads((ROOT / 'table_tilt_20261010_053815_726640' /
                       'tilt_result.json').read_text())
    reg = json.loads((ROOT / 'sac_cube_rim_register_1010_stage0_result.json').read_text())
    trial = json.loads((ROOT / 'leap_support_capture_result.json').read_text())
    actual = [s for s in trial['samples'] if s['phase'] == 'place_cube'][-1]['actual']
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    model.opt.gravity[:] = tilt['gravity_in_sim_world_m_s2_candidate']
    cube_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
    q[:16] = hw_to_sim(np.asarray(actual) * 2 * np.pi / 4096)
    q[-4:] = reg['cube_quat_world_candidate']
    center = np.asarray(reg['cube_center_world_m'], dtype=float)
    rows = []
    for dx in (-5., -2.5, 0., 2.5, 5.):
        for dy in (-5., -2.5, 0., 2.5, 5.):
            for dz in (-5., -2.5, 0., 2.5, 5.):
                q[-7:-4] = center + np.array([dx, dy, dz]) / 1000
                rows.append({'offset_mm': [dx, dy, dz], **probe(model, q, cube_geom)})
    plausible = [r for r in rows if r['penetration_mm'] <= 2 and
                 r['drift_mm'] <= 10 and r['initial_geoms'] and r['final_geoms']]
    report = {
        'status': 'offline_measured_gravity_local_search',
        'hardware_authorized': False,
        'start_center_m': center.tolist(),
        'sample_count': len(rows),
        'plausible_contact_count': len(plausible),
        'lowest_drift': sorted(rows, key=lambda r: r['drift_mm'])[:10],
        'plausible': sorted(plausible, key=lambda r: r['drift_mm']),
        'caveat': 'Measured gravity has a candidate marker-to-model transform. Local pose sensitivity is not a calibrated cube reset.',
    }
    (ROOT / 'sac_measured_gravity_local_search_result.json').write_text(
        json.dumps(report, indent=2) + '\n')
    print(json.dumps({'status': report['status'], 'sample_count': len(rows),
                      'plausible_contact_count': len(plausible),
                      'lowest_drift': report['lowest_drift'][:3],
                      'plausible': report['plausible'][:3]}, indent=2))


if __name__ == '__main__':
    main()
