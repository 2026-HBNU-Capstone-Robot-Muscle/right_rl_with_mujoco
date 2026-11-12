"""Compare MuJoCo grasp site, palm sensor, and saved marker registration offline."""

import json
from pathlib import Path
import sys

import jax
import mujoco
from mujoco import mjx
import numpy as np
from mujoco_playground import registry

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from hand_registration_geometry import candidate


def main():
    config = registry.get_default_config("LeapCubeFaceUp")
    config.impl = "jax"
    env = registry.load("LeapCubeFaceUp", config=config)
    state = env.reset(jax.random.PRNGKey(0))
    model = env.mj_model
    data = mujoco.MjData(model)
    data.qpos[:] = np.asarray(state.data.qpos)
    mujoco.mj_forward(model, data)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_site")
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "palm_position")
    adr = model.sensor_adr[sensor_id]
    dim = model.sensor_dim[sensor_id]
    reg = candidate()
    cube_in_marker = np.asarray(json.loads((ROOT / "sac_success_cube_poses.json").read_text(encoding="utf-8"))[0]["cube_minus_hand_marker_m"])
    R_world_marker = np.asarray(reg["hand_to_world_candidate"])
    grasp_in_marker = np.asarray(reg["grasp_site_in_marker_m_candidate"])
    cube_offset_world = R_world_marker @ (cube_in_marker - grasp_in_marker)
    mjx_forward = mjx.forward(env.mjx_model, state.data)
    step_state = jax.jit(env.step)(state, np.zeros(16, dtype=np.float32))
    report = {
        "site_local_pos_m": model.site_pos[site_id].tolist(),
        "site_cpu_world_m": data.site_xpos[site_id].tolist(),
        "sensor_cpu_world_m": data.sensordata[adr:adr + dim].tolist(),
        "sensor_mjx_world_m": np.asarray(env.get_palm_position(state.data)).tolist(),
        "site_mjx_reset_world_m": np.asarray(state.data.site_xpos[site_id]).tolist(),
        "site_mjx_forward_world_m": np.asarray(mjx_forward.site_xpos[site_id]).tolist(),
        "sensor_mjx_forward_world_m": np.asarray(env.get_palm_position(mjx_forward)).tolist(),
        "site_mjx_after_step_world_m": np.asarray(step_state.data.site_xpos[site_id]).tolist(),
        "sensor_mjx_after_step_world_m": np.asarray(env.get_palm_position(step_state.data)).tolist(),
        "marker_cube_relative_world_m": cube_offset_world.tolist(),
        "cube_world_using_cpu_site_m": (data.site_xpos[site_id] + cube_offset_world).tolist(),
        "cube_world_using_mjx_reset_sensor_m": (np.asarray(env.get_palm_position(state.data)) + cube_offset_world).tolist(),
        "cube_world_using_mjx_forward_sensor_m": (np.asarray(env.get_palm_position(mjx_forward)) + cube_offset_world).tolist(),
    }
    (ROOT / "sac_frame_diagnostic_result.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
