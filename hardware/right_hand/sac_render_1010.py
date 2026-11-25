"""Render the October 10 measured first pose in MuJoCo, without device access."""

import json
from pathlib import Path

import jax
import mujoco
import numpy as np
from mujoco_playground import registry

from sac_cube_model import set_cube_physics
from sac_render_geometry import render
from leap_hw import hw_to_sim

ROOT = Path(__file__).resolve().parent


def main():
    registration = json.loads((ROOT/'sac_cube_rim_register_1010_stage0_result.json').read_text())
    trial = json.loads((ROOT/'leap_support_capture_result.json').read_text())
    samples = [s for s in trial['samples'] if s['phase'] == 'place_cube']
    config = registry.get_default_config('LeapCubeFaceUp')
    config.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=config)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    cube_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    model.geom_rgba[cube_geom] = [1., .25, .1, 1.]
    q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
    q[:16] = hw_to_sim(np.asarray(samples[-1]['actual']) * 2*np.pi/4096)
    q[-7:-4] = registration['cube_center_world_m']
    q[-4:] = registration['cube_quat_world_candidate']
    output = ROOT/'sac_measured_1010_collision.png'
    render(model, q, output, True)
    print(output)


if __name__ == '__main__':
    main()
