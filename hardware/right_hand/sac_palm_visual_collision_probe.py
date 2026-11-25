"""Offline probe: does the palm visual mesh explain support better than coarse boxes?"""
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
    q[-7:-4] = anchor['cube_center_in_sim_world_candidate_m']
    q[-4:] = anchor['cube_quat_world_candidate']
    cube = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    visual = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'palm_visual')
    boxes = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'palm_collision_' + str(i))
             for i in (6, 7, 8)]
    rows = []
    for setting in ('original', 'visual_plus_boxes', 'visual_without_boxes', 'without_boxes'):
        model.geom_contype[visual] = model.geom_conaffinity[visual] = int(setting.startswith('visual'))
        for box in boxes:
            model.geom_contype[box] = model.geom_conaffinity[box] = int('without_boxes' not in setting)
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
        rows.append({'setting': setting, 'initial_contacts': len(initial),
                     'initial_geoms': sorted({v[1] for v in initial}),
                     'initial_penetration_mm': -1000 * min([0.] + [v[0] for v in initial]),
                     'final_contacts': len(final),
                     'final_geoms': sorted({v[1] for v in final}),
                     'drift_mm': float(1000 * np.linalg.norm(data.qpos[-7:-4] - q[-7:-4]))})
    output = {'rows': rows, 'hardware_authorized': False,
              'caveat': ('Compiled MuJoCo broadphase/pair filtering may not update when geom flags are '
                         'changed after model compilation. Zero palm_visual contacts therefore do not '
                         'establish that its mesh cannot support the cube. A triangle mesh may also be '
                         'treated as a convex hull; this is only a model sensitivity probe.')}
    (ROOT / 'sac_palm_visual_collision_probe_result.json').write_text(json.dumps(output, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(output, indent=2))


if __name__ == '__main__':
    main()
