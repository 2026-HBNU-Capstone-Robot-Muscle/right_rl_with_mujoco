"""Find stage-0 cube starts supported by modeled fingers; offline diagnostic."""

import json
from pathlib import Path
import sys

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
    reset = env.reset(jax.random.PRNGKey(0))
    base = np.asarray(reset.data.qpos, dtype=float).copy()
    audit = json.loads((ROOT / 'sac_stage0_alignment_audit_result.json').read_text(encoding='utf-8'))
    base[:16] = hw_to_sim(np.asarray(audit['loaded_actual_ticks']) * 2 * np.pi / 4096)
    registration = json.loads((ROOT / 'sac_registration_anchor_result.json').read_text(encoding='utf-8'))
    observed = np.asarray(registration['cube_center_in_sim_world_candidate_m'], dtype=float)
    quat = np.asarray(registration['cube_quat_world_candidate'], dtype=float)
    cube_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'cube')
    body_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or ''
                  for i in range(model.nbody)]

    def contacts(data):
        finger, palm, deepest = set(), 0, 0.
        for contact in data.contact:
            body1 = int(model.geom_bodyid[contact.geom1])
            body2 = int(model.geom_bodyid[contact.geom2])
            if cube_body not in (body1, body2):
                continue
            other = body2 if body1 == cube_body else body1
            name = body_names[other]
            if name.startswith(('if_', 'mf_', 'rf_', 'th_')):
                finger.add(name.split('_')[0])
            elif name == 'palm':
                palm += 1
            deepest = min(deepest, float(contact.dist))
        return sorted(finger), palm, deepest

    rows = []
    for x in np.linspace(.09, .16, 8):
        for y in np.linspace(-.06, .00, 7):
            for z in np.linspace(.00, .07, 8):
                data = mujoco.MjData(model)
                data.qpos[:] = base
                data.qpos[-7:-4] = (x, y, z)
                data.qpos[-4:] = quat
                data.ctrl[:] = base[:16]
                mujoco.mj_forward(model, data)
                before_fingers, before_palm, penetration = contacts(data)
                for _ in range(30):
                    mujoco.mj_step(model, data)
                after_fingers, after_palm, _ = contacts(data)
                point = np.array([x, y, z])
                rows.append(dict(start_m=point.tolist(), end_m=data.qpos[-7:-4].tolist(),
                                 offset_from_image_mm=float(1000 * np.linalg.norm(point - observed)),
                                 drift_mm=float(1000 * np.linalg.norm(data.qpos[-7:-4] - point)),
                                 penetration_mm=float(-1000 * penetration),
                                 initial_fingers=before_fingers, final_fingers=after_fingers,
                                 initial_palm_contacts=before_palm, final_palm_contacts=after_palm))
    supported = [r for r in rows if r['initial_fingers'] and r['final_fingers']
                 and r['penetration_mm'] < 4 and r['drift_mm'] < 15]
    supported.sort(key=lambda r: (r['offset_from_image_mm'], r['drift_mm']))
    finger_contact = [r for r in rows if r['initial_fingers'] and r['final_fingers']]
    finger_contact.sort(key=lambda r: (r['drift_mm'], r['penetration_mm']))
    result = dict(hardware_authorized=False, samples=len(rows),
                  measured_start_m=observed.tolist(),
                  finger_supported_under_threshold=len(supported),
                  nearest_finger_supported=supported[:20],
                  lowest_drift_with_finger_contact=finger_contact[:20],
                  caveat='Coarse collision diagnostic; contact validity and real-to-sim registration unverified.')
    path = ROOT / 'sac_finger_support_search_result.json'
    path.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k not in
                      ('nearest_finger_supported', 'lowest_drift_with_finger_contact')}, indent=2))
    print('nearest_finger_supported', supported[:3])
    print('lowest_drift_with_finger_contact', finger_contact[:3])


if __name__ == '__main__':
    main()
