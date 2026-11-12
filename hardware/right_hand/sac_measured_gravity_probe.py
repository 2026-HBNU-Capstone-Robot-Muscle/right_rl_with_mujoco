"""Motor-free contact check using tabletop-measured gravity in candidate sim frame."""

import json
import sys
from pathlib import Path

import jax
import mujoco
import numpy as np
from mujoco_playground import registry

from sac_cube_model import set_cube_physics
from sac_index_middle_support_probe import probe
from sac_marker_plane_support_probe import RUNS

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from leap_hw import hw_to_sim


def main():
    tilt_path = ROOT / 'table_tilt_20261010_053815_726640' / 'tilt_result.json'
    tilt = json.loads(tilt_path.read_text())
    gravity = np.asarray(tilt['gravity_in_sim_world_m_s2_candidate'], dtype=float)
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    cube_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    rows = []
    for label, reg_file, trial_file in RUNS:
        reg = json.loads((ROOT / reg_file).read_text())
        trial = json.loads((ROOT / trial_file).read_text())
        actual = [s for s in trial['samples'] if s['phase'] == 'place_cube'][-1]['actual']
        q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
        q[:16] = hw_to_sim(np.asarray(actual) * 2 * np.pi / 4096)
        q[-7:-4] = reg['cube_center_world_m']
        q[-4:] = reg['cube_quat_world_candidate']
        for mode, vector in [('default', [0., 0., -9.81]), ('measured_candidate', gravity)]:
            model.opt.gravity[:] = vector
            rows.append({'run': label, 'gravity_mode': mode, **probe(model, q, cube_geom)})
    report = {
        'status': 'offline_measured_gravity_contact_probe',
        'hardware_authorized': False,
        'tilt_result': str(tilt_path),
        'marker_table_angle_deg': tilt['marker_table_angle_deg'],
        'gravity_in_sim_world_m_s2_candidate': gravity.tolist(),
        'results': rows,
        'caveat': 'Measured tabletop/marker angle; marker-to-simulator orientation and cube registration remain provisional. A 0.3 s fixed-goal hold is not a policy rollout.',
    }
    out = ROOT / 'sac_measured_gravity_probe_result.json'
    out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'status': report['status'], 'marker_table_angle_deg': report['marker_table_angle_deg'],
                      'gravity': report['gravity_in_sim_world_m_s2_candidate'],
                      'results': [{'run': r['run'], 'mode': r['gravity_mode'],
                                   'penetration_mm': r['penetration_mm'], 'drift_mm': r['drift_mm']}
                                  for r in rows]}, indent=2))


if __name__ == '__main__':
    main()
