"""Offline sensitivity of the measured cube start to hand/cube friction and depth."""

import json
from pathlib import Path
import sys

import jax
import mujoco
import numpy as np
from mujoco_playground import registry

from hand_registration_geometry import candidate
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
    reset = env.reset(jax.random.PRNGKey(0))
    q = np.asarray(reset.data.qpos, dtype=float).copy()
    audit = json.loads((ROOT / 'sac_stage0_alignment_audit_result.json').read_text(encoding='utf-8'))
    q[:16] = hw_to_sim(np.asarray(audit['loaded_actual_ticks']) * 2 * np.pi / 4096)
    registration = json.loads((ROOT / 'sac_registration_anchor_result.json').read_text(encoding='utf-8'))
    center = np.asarray(registration['cube_center_in_sim_world_candidate_m'], dtype=float)
    quat = np.asarray(registration['cube_quat_world_candidate'], dtype=float)
    normal = np.asarray(candidate()['hand_to_world_candidate'])[:, 2]
    body_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or ''
                  for i in range(model.nbody)]
    hand_geoms = [i for i in range(model.ngeom) if body_names[int(model.geom_bodyid[i])]
                  .startswith(('palm', 'if_', 'mf_', 'rf_', 'th_'))]
    cube_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    original = model.geom_friction.copy()
    rows = []
    for friction in (.2, .5, 1., 2.):
        model.geom_friction[:] = original
        model.geom_friction[hand_geoms, 0] = friction
        model.geom_friction[cube_geom, 0] = friction
        for lift_mm in (0, 5, 10, 15):
            start = center + normal * (lift_mm / 1000)
            data = mujoco.MjData(model)
            data.qpos[:] = q
            data.qpos[-7:-4] = start
            data.qpos[-4:] = quat
            data.ctrl[:] = q[:16]
            mujoco.mj_forward(model, data)
            overlap = float(min([0.] + [contact.dist for contact in data.contact
                                      if cube_geom in (contact.geom1, contact.geom2)]))
            for _ in range(30):
                mujoco.mj_step(model, data)
            rows.append(dict(friction=float(friction), lift_along_marker_normal_mm=lift_mm,
                             initial_penetration_mm=-1000 * overlap,
                             cube_drift_mm=float(1000 * np.linalg.norm(data.qpos[-7:-4] - start)),
                             end_z_m=float(data.qpos[-7 + 2])))
    result = dict(hardware_authorized=False, rows=rows,
                  caveat='In-memory parameter sensitivity only; friction not calibrated from real contact data.')
    (ROOT / 'sac_friction_sensitivity_result.json').write_text(
        json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
