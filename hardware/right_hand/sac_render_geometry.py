"""Render default and camera-measured LEAP cube placements; no device I/O."""

import json
import os
from pathlib import Path
import sys

os.environ.setdefault("MUJOCO_GL", "egl")

import jax
import mujoco
import numpy as np
from PIL import Image
from mujoco_playground import registry
from sac_cube_model import set_cube_physics

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / "work" / "Leap_Test" / "deploy"))
from leap_hw import hw_to_sim


def render(model, qpos, path, collision_only=False):
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    data.ctrl[:] = qpos[:16]
    mujoco.mj_forward(model, data)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [0.09, 0.0, 0.01]
    camera.distance = 0.30
    camera.azimuth = 130
    camera.elevation = -55
    with mujoco.Renderer(model, height=720, width=960) as renderer:
        option = mujoco.MjvOption()
        if collision_only:
            option.geomgroup[2] = 0  # hide the original 70 mm cube visual mesh
            option.geomgroup[3] = 1  # show the corrected collision box
        renderer.update_scene(data, camera=camera, scene_option=option)
        Image.fromarray(renderer.render()).save(path)


def main():
    config = registry.get_default_config("LeapCubeFaceUp")
    config.impl = "jax"
    env = registry.load("LeapCubeFaceUp", config=config)
    state = env.reset(jax.random.PRNGKey(0))
    original = np.asarray(state.data.qpos, dtype=float)
    render(env.mj_model, original, ROOT / "sac_sim_default.png")

    raw_ticks = json.loads((ROOT / "sequence_flip_poses.json").read_text(encoding="utf-8"))["poses"][0]["raw_ticks"]
    observed = json.loads((ROOT / "sac_success_cube_poses.json").read_text(encoding="utf-8"))[0]
    measured = original.copy()
    measured[:16] = hw_to_sim(np.asarray(raw_ticks) * 2 * np.pi / 4096)
    measured[-7:-4] = -np.asarray(observed["palm_minus_cube_m"])
    measured[-4:] = observed["cube_quat_world"]
    render(env.mj_model, measured, ROOT / "sac_sim_measured.png")
    set_cube_physics(env.mj_model, mass_kg=0.100)
    cube_geom = mujoco.mj_name2id(env.mj_model, mujoco.mjtObj.mjOBJ_GEOM, "cube")
    env.mj_model.geom_rgba[cube_geom] = [1.0, 0.25, 0.1, 1.0]
    measured[-7:-4] += [0.11, 0.0, 0.03]  # correct reset-time palm sensor = 0 bug
    render(env.mj_model, measured, ROOT / "sac_sim_measured_corrected_collision.png", True)
    fitted = json.loads((ROOT / "sac_fit_contact_real_cube_100g_result.json").read_text(encoding="utf-8"))[
        "contact_retained_top_candidates"
    ][0]
    contact = measured.copy()
    contact[-7:-4] = fitted["start_m"]
    render(env.mj_model, contact, ROOT / "sac_sim_contact_candidate_collision.png", True)
    print(json.dumps({"default": str(ROOT / "sac_sim_default.png"),
                      "measured": str(ROOT / "sac_sim_measured.png")}))


if __name__ == "__main__":
    main()
