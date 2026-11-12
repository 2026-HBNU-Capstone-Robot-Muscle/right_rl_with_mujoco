"""Test whether the measured cube start is falsely supported by palm collision boxes."""

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
    state = env.reset(jax.random.PRNGKey(0))
    q = np.asarray(state.data.qpos, dtype=float).copy()
    audit = json.loads((ROOT / 'sac_stage0_alignment_audit_result.json').read_text(encoding='utf-8'))
    q[:16] = hw_to_sim(np.asarray(audit['loaded_actual_ticks']) * 2 * np.pi / 4096)
    placement = json.loads((ROOT / 'sac_registration_anchor_result.json').read_text(encoding='utf-8'))
    q[-7:-4] = placement['cube_center_in_sim_world_candidate_m']
    q[-4:] = placement['cube_quat_world_candidate']
    cube_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'cube')
    original_type = model.geom_contype.copy()
    original_affinity = model.geom_conaffinity.copy()
    rows = []
    variants = ((), (6,), (7,), (8,), (6, 7), (6, 8), (7, 8),
                (6, 7, 8), tuple(range(1, 11)))
    for disabled in variants:
        model.geom_contype[:] = original_type
        model.geom_conaffinity[:] = original_affinity
        for suffix in disabled:
            name = f'palm_collision_{suffix}'
            geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
            if geom >= 0:
                model.geom_contype[geom] = 0
                model.geom_conaffinity[geom] = 0
        data = mujoco.MjData(model)
        data.qpos[:] = q
        data.ctrl[:] = q[:16]
        mujoco.mj_forward(model, data)
        def cube_contacts():
            found = []
            for contact in data.contact:
                body1 = int(model.geom_bodyid[contact.geom1])
                body2 = int(model.geom_bodyid[contact.geom2])
                if cube_body not in (body1, body2):
                    continue
                other = int(contact.geom2 if body1 == cube_body else contact.geom1)
                found.append((float(contact.dist),
                              mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other)))
            return found
        initial = cube_contacts()
        for _ in range(30):
            mujoco.mj_step(model, data)
        final = cube_contacts()
        rows.append(dict(disabled_palm_collision_ids=list(disabled),
                         initial_cube_contacts=len(initial),
                         initial_penetration_mm=-1000 * min([0.] + [c[0] for c in initial]),
                         final_cube_contacts=len(final),
                         final_contact_geoms=sorted(set(c[1] for c in final)),
                         drift_mm=float(1000 * np.linalg.norm(data.qpos[-7:-4] - q[-7:-4])),
                         final_center_m=data.qpos[-7:-4].tolist()))
    result = dict(hardware_authorized=False, rows=rows,
                  caveat='In-memory collision ablation for diagnosis only; never a hardware setting.')
    (ROOT / 'sac_palm_collision_ablation_result.json').write_text(
        json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
