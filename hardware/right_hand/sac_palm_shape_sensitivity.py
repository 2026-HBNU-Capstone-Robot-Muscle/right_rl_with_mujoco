"""Offline sensitivity of coarse palm collision boxes 6–8 at measured starts."""
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
    base_q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
    old = json.loads((ROOT / 'sac_registration_anchor_result.json').read_text(encoding='utf-8'))
    new = json.loads((ROOT / 'sac_registration_support_photo_diagnostic.json').read_text(encoding='utf-8'))
    old_ticks = json.loads((ROOT / 'sac_stage0_alignment_audit_result.json').read_text(encoding='utf-8'))['loaded_actual_ticks']
    trial = json.loads((ROOT / 'leap_support_capture_result.json').read_text(encoding='utf-8'))
    new_ticks = [row for row in trial['samples'] if row['phase']=='place_cube'][-1]['actual']
    geom_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f'palm_collision_{i}')
                for i in (6, 7, 8)]
    original_sizes = model.geom_size[geom_ids].copy()
    cube_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'cube')
    rows = []
    for label, placement, ticks in (('old', old, old_ticks), ('latest', new, new_ticks)):
        q = base_q.copy()
        q[:16] = hw_to_sim(np.asarray(ticks, dtype=float)*2*np.pi/4096)
        q[-7:-4] = placement['cube_center_in_sim_world_candidate_m']
        q[-4:] = placement['cube_quat_world_candidate']
        for scale in (1.0, .95, .90, .85, .80, .75, .70, .60, .50):
            model.geom_size[geom_ids] = original_sizes*scale
            data = mujoco.MjData(model)
            data.qpos[:] = q
            data.ctrl[:] = q[:16]
            mujoco.mj_forward(model, data)
            def cube_contacts():
                found = []
                for item in data.contact:
                    a, b = model.geom_bodyid[item.geom1], model.geom_bodyid[item.geom2]
                    if cube_body in (a, b):
                        other = item.geom2 if a == cube_body else item.geom1
                        found.append((float(item.dist), mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other)))
                return found
            initial = cube_contacts()
            for _ in range(30):
                mujoco.mj_step(model, data)
            final = cube_contacts()
            rows.append({'pose': label, 'scale': scale,
                         'initial_penetration_mm': -1000*min([0.]+[item[0] for item in initial]),
                         'initial_contacts': len(initial),
                         'initial_geoms': sorted({item[1] for item in initial}),
                         'final_contacts': len(final),
                         'final_geoms': sorted({item[1] for item in final}),
                         'drift_mm': float(1000*np.linalg.norm(data.qpos[-7:-4]-q[-7:-4]))})
    model.geom_size[geom_ids] = original_sizes
    result = {'original_half_extents_m': {str(i): original_sizes[k].tolist()
                                         for k,i in enumerate((6,7,8))},
              'rows': rows, 'hardware_authorized': False,
              'note': 'In-memory shape sensitivity only; physical palm shape not yet measured.'}
    (ROOT / 'sac_palm_shape_sensitivity_result.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
