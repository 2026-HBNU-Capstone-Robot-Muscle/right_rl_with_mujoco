"""Render the measured collision cube against the visible hand mesh; offline."""

import json
import os
from pathlib import Path
import sys

os.environ.setdefault('MUJOCO_GL', 'egl')

import jax
import mujoco
import numpy as np
from PIL import Image
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
    data = mujoco.MjData(model)
    data.qpos[:] = np.asarray(reset.data.qpos, dtype=float)
    audit = json.loads((ROOT / 'sac_stage0_alignment_audit_result.json').read_text(encoding='utf-8'))
    data.qpos[:16] = hw_to_sim(np.asarray(audit['loaded_actual_ticks']) * 2 * np.pi / 4096)
    placement = json.loads((ROOT / 'sac_registration_anchor_result.json').read_text(encoding='utf-8'))
    data.qpos[-7:-4] = placement['cube_center_in_sim_world_candidate_m']
    data.qpos[-4:] = placement['cube_quat_world_candidate']
    mujoco.mj_forward(model, data)
    cube_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'cube')
    cube_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    for i in range(model.ngeom):
        if model.geom_bodyid[i] == cube_body and i != cube_geom:
            model.geom_group[i] = 5
    model.geom_group[cube_geom] = 0
    model.geom_rgba[cube_geom] = [1., .25, .1, .70]
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0  # hide simplified hand collision boxes
    option.geomgroup[5] = 0  # hide original 70 mm visual cube
    for suffix, azimuth, elevation in (('angled', 130, -55), ('side', 90, -15)):
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat[:] = [.11, 0, 0]
        camera.distance = .30
        camera.azimuth = azimuth
        camera.elevation = elevation
        with mujoco.Renderer(model, height=720, width=960) as renderer:
            renderer.update_scene(data, camera=camera, scene_option=option)
            Image.fromarray(renderer.render()).save(ROOT / f'sac_visual_contact_{suffix}.png')
    print('Saved visual-mesh contact renders; no device I/O')


if __name__ == '__main__':
    main()
