"""Test whether the installed palm visual mesh explains missing cube support.

Offline MuJoCo diagnostic only. The source MJCF is left unchanged.
"""

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
sys.path.insert(0, str(ROOT.parent/'work/Leap_Test/deploy'))
from leap_hw import hw_to_sim


def main():
    reg = json.loads((ROOT/'sac_cube_rim_register_1010_stage0_result.json').read_text())
    trial = json.loads((ROOT/'leap_support_capture_result.json').read_text())
    actual = [r for r in trial['samples'] if r['phase']=='place_cube'][-1]['actual']
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    cube_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    palm_visual = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'palm_visual')
    palm_box = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'palm_collision_1')
    q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
    q[:16] = hw_to_sim(np.asarray(actual)*2*np.pi/4096)
    q[-7:-4] = reg['cube_center_world_m']
    q[-4:] = reg['cube_quat_world_candidate']
    baseline = probe(model, q, cube_id)
    before = {'contype': int(model.geom_contype[palm_visual]),
              'conaffinity': int(model.geom_conaffinity[palm_visual]),
              'friction': model.geom_friction[palm_visual].tolist()}
    model.geom_contype[palm_visual] = model.geom_contype[palm_box]
    model.geom_conaffinity[palm_visual] = model.geom_conaffinity[palm_box]
    model.geom_friction[palm_visual] = model.geom_friction[palm_box]
    mujoco.mj_setConst(model, mujoco.MjData(model))
    enabled = probe(model, q, cube_id)
    data = mujoco.MjData(model)
    data.qpos[:] = q
    data.ctrl[:] = q[:16]
    mujoco.mj_forward(model, data)
    visual_gap = float(mujoco.mj_geomDistance(model, data, cube_id, palm_visual, 1.0, None))
    box_gaps = {}
    for index in range(1, 11):
        name = f'palm_collision_{index}'
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom_id >= 0:
            box_gaps[name] = float(mujoco.mj_geomDistance(model, data, cube_id, geom_id, 1.0, None))
    report = {'status': 'offline_visual_mesh_contact_probe', 'hardware_authorized': False,
              'visual_before': before, 'baseline': baseline,
              'palm_visual_collision_enabled_in_memory': enabled,
              'palm_visual_signed_distance_mm': visual_gap*1000,
              'palm_box_signed_distances_mm': {k: v*1000 for k,v in box_gaps.items()},
              'caveat': 'The visual mesh can have inaccurate convex contact; this probe does not validate a physics model.'}
    (ROOT/'sac_visual_palm_contact_probe_1010_result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
