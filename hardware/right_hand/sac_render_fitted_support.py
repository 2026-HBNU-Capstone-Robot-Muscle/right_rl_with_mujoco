"""Render photographed and locally fitted cube starts in a recompiled model."""
import json
import os
import sys
from pathlib import Path

os.environ.setdefault('MUJOCO_GL', 'egl')
import jax
import mujoco
import numpy as np
from PIL import Image
from mujoco_playground import registry

from sac_palm_recompiled_collision_probe import make_model


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from leap_hw import hw_to_sim


def main():
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model, _ = make_model(env, 'visual_without_boxes')
    model.vis.global_.offwidth = 960
    model.vis.global_.offheight = 720
    q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
    trial = json.loads((ROOT / 'leap_support_capture_result.json').read_text(encoding='utf-8'))
    ticks = [r for r in trial['samples'] if r['phase'] == 'place_cube'][-1]['actual']
    q[:16] = hw_to_sim(np.asarray(ticks) * 2 * np.pi / 4096)
    anchor = json.loads((ROOT / 'sac_registration_support_photo_diagnostic.json').read_text(encoding='utf-8'))
    q[-4:] = anchor['cube_quat_world_candidate']
    center = np.asarray(anchor['cube_center_in_sim_world_candidate_m'], dtype=float)
    cube_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'cube')
    cube_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    for i in range(model.ngeom):
        if model.geom_bodyid[i] == cube_body and i != cube_geom:
            model.geom_group[i] = 5
    model.geom_group[cube_geom] = 0
    model.geom_rgba[cube_geom] = [1., .25, .1, .70]
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0
    option.geomgroup[5] = 0
    for label, delta_mm in (('photographed_candidate', (0, 0, 0)),
                            ('fitted_candidate', (14, 0, -2))):
        data = mujoco.MjData(model)
        data.qpos[:] = q
        data.qpos[-7:-4] = center + np.asarray(delta_mm) / 1000
        mujoco.mj_forward(model, data)
        for suffix, azimuth, elevation in (('side', 90, -15), ('angled', 130, -55)):
            camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            camera.lookat[:] = [.11, 0, 0]
            camera.distance = .30
            camera.azimuth = azimuth
            camera.elevation = elevation
            with mujoco.Renderer(model, height=720, width=960) as renderer:
                renderer.update_scene(data, camera=camera, scene_option=option)
                Image.fromarray(renderer.render()).save(ROOT / f'sac_{label}_{suffix}.png')
    print('Saved photographed and fitted offline renders; no device I/O')


if __name__ == '__main__':
    main()
