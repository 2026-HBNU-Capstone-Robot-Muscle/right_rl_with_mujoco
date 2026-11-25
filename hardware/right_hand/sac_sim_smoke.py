"""Check the installed LEAP cube simulator and exported policy (no motor I/O)."""

import json
from pathlib import Path
import sys

import jax
import jax.numpy as jnp
import mujoco
import numpy as np
from mujoco import mjx
from mujoco_playground import registry
from sac_cube_model import set_cube_physics

PROJECT = Path(__file__).resolve().parent.parent / "work" / "Leap_Test"
sys.path.insert(0, str(PROJECT / "deploy"))
from policy_np import Policy
from leap_hw import hw_to_sim


def main():
    root = Path(__file__).resolve().parent
    registration_file = None
    if '--registration-file' in sys.argv:
        registration_file = sys.argv[sys.argv.index('--registration-file') + 1]
    trial_file = None
    if '--trial-file' in sys.argv:
        trial_file = sys.argv[sys.argv.index('--trial-file') + 1]
    output_file = None
    if '--output-file' in sys.argv:
        output_file = sys.argv[sys.argv.index('--output-file') + 1]
    gravity_file = None
    if '--gravity-file' in sys.argv:
        gravity_file = sys.argv[sys.argv.index('--gravity-file') + 1]
    offset_mm = np.zeros(3)
    if '--offset-mm' in sys.argv:
        start = sys.argv.index('--offset-mm') + 1
        offset_mm = np.asarray([float(x) for x in sys.argv[start:start + 3]])
        if offset_mm.shape != (3,):
            raise ValueError('--offset-mm requires three numbers')
    name = "LeapCubeFaceUp"
    config = registry.get_default_config(name)
    config.impl = "jax"
    target_face = None
    if "--target-face" in sys.argv:
        target_face = int(sys.argv[sys.argv.index("--target-face") + 1])
        if target_face not in range(6):
            raise ValueError("--target-face must be 0..5")
        config.target_face = target_face
    env = registry.load(name, config=config)
    real_cube = "--real-cube" in sys.argv
    cube_mass_g = None
    hold_steps = 0
    if "--hold-steps" in sys.argv:
        hold_steps = int(sys.argv[sys.argv.index("--hold-steps") + 1])
        if hold_steps < 0:
            raise ValueError("--hold-steps must be nonnegative")
    if "--cube-mass-g" in sys.argv:
        cube_mass_g = float(sys.argv[sys.argv.index("--cube-mass-g") + 1])
        if cube_mass_g <= 0 or not real_cube:
            raise ValueError("--cube-mass-g requires --real-cube and a positive mass")
    cube_physics = None
    if real_cube:
        if cube_mass_g is None:
            cube_geom_id = mujoco.mj_name2id(env.mj_model, mujoco.mjtObj.mjOBJ_GEOM, "cube")
            env.mj_model.geom_size[cube_geom_id, :3] = 0.0545 / 2
            mujoco.mj_setConst(env.mj_model, mujoco.MjData(env.mj_model))
        else:
            cube_physics = set_cube_physics(env.mj_model, mass_kg=cube_mass_g / 1000)
        if gravity_file is not None:
            tilt = json.loads((root / gravity_file).read_text(encoding='utf-8'))
            env.mj_model.opt.gravity[:] = tilt['gravity_in_sim_world_m_s2_candidate']
        env._mjx_model = mjx.put_model(env.mj_model, impl="jax")
    elif gravity_file is not None:
        raise ValueError('--gravity-file requires --real-cube')
    state = env.reset(jax.random.PRNGKey(0))
    # make_data in reset has not populated frame sensors until a forward/step.
    # Use the populated grasp-site position for camera-to-simulator placement.
    reference_data = mjx.forward(env.mjx_model, state.data)
    obs = state.obs
    poses = json.loads(
        (Path(__file__).resolve().parent / "sequence_flip_poses.json").read_text(
            encoding="utf-8"
        )
    )
    recorded_start = hw_to_sim(
        np.asarray(poses["poses"][0]["raw_ticks"], dtype=float) * (2 * np.pi / 4096)
    )
    if trial_file is not None:
        trial = json.loads((root / trial_file).read_text(encoding='utf-8'))
        placed = [row for row in trial['samples'] if row['phase'] == 'place_cube']
        if not placed:
            raise RuntimeError('Trial has no measured place_cube joint sample')
        recorded_start = hw_to_sim(
            np.asarray(placed[-1]['actual'], dtype=float) * (2 * np.pi / 4096))
    sim_start = np.asarray(state.data.qpos[:16], dtype=float)
    result = {
        "environment": name,
        "target_face_config": target_face,
        "real_cube_collision_size_applied": real_cube,
        "cube_physics": cube_physics,
        "gravity_input": gravity_file,
        "gravity_sim_m_s2": np.asarray(env.mj_model.opt.gravity).tolist(),
        "start_offset_mm": offset_mm.tolist(),
        "jax_devices": [str(device) for device in jax.devices()],
        "action_size": int(env.action_size),
        "observation": (
            {key: list(value.shape) for key, value in obs.items()}
            if isinstance(obs, dict)
            else list(obs.shape)
        ),
        "cube_position": [float(x) for x in state.data.qpos[-7:-4]],
        "palm_position": [float(x) for x in env.get_palm_position(reference_data)],
        "cube_quaternion": [float(x) for x in state.data.qpos[-4:]],
        "target_quaternion": [float(x) for x in state.data.mocap_quat[0]],
        "start_pose_comparison": {
            "sim_joint_rad": sim_start.tolist(),
            "recorded_joint_rad": recorded_start.tolist(),
            "max_abs_gap_rad": float(np.max(np.abs(sim_start - recorded_start))),
            "mean_abs_gap_rad": float(np.mean(np.abs(sim_start - recorded_start))),
        },
        "config": {
            key: float(getattr(config, key))
            for key in ("ctrl_dt", "action_scale", "position_threshold")
            if hasattr(config, key)
        },
        "info_keys": list(state.info),
    }
    contact_candidate = "--contact-candidate" in sys.argv
    fitted_contact = "--fitted-contact" in sys.argv or contact_candidate
    if "--recorded-start" in sys.argv or fitted_contact or registration_file is not None:
        qpos = state.data.qpos.at[:16].set(jnp.asarray(recorded_start, dtype=jnp.float32))
        if registration_file is not None:
            reg = json.loads((root / registration_file).read_text(encoding='utf-8'))
            center = reg.get('cube_center_world_m', reg.get('cube_center_in_sim_world_candidate_m'))
            qpos = qpos.at[-7:-4].set(jnp.asarray(np.asarray(center) + offset_mm / 1000,
                                                  dtype=jnp.float32))
            qpos = qpos.at[-4:].set(jnp.asarray(reg['cube_quat_world_candidate'], dtype=jnp.float32))
            result['registration_input'] = registration_file
            result['trial_input'] = trial_file
        if fitted_contact:
            fit = json.loads(
                (Path(__file__).resolve().parent / (
                    f"sac_fit_contact_real_cube_{cube_mass_g:g}g_result.json"
                    if cube_mass_g is not None else "sac_fit_contact_real_cube_result.json"
                )).read_text(encoding="utf-8")
            )[("contact_retained_top_candidates" if contact_candidate else "top_candidates")][0]
            measured = json.loads(
                (Path(__file__).resolve().parent / "sac_success_cube_poses.json").read_text(encoding="utf-8")
            )[0]
            qpos = qpos.at[-7:-4].set(jnp.asarray(fit["start_m"], dtype=jnp.float32))
            qpos = qpos.at[-4:].set(jnp.asarray(measured["cube_quat_world"], dtype=jnp.float32))
            result["fitted_contact_input"] = fit
        if "--measured-cube" in sys.argv:
            measured = json.loads(
                (Path(__file__).resolve().parent / "sac_success_cube_poses.json").read_text(
                    encoding="utf-8"
                )
            )[0]
            palm = np.asarray(env.get_palm_position(reference_data), dtype=float)
            cube_pos = palm - np.asarray(measured["palm_minus_cube_m"], dtype=float)
            qpos = qpos.at[-7:-4].set(jnp.asarray(cube_pos, dtype=jnp.float32))
            qpos = qpos.at[-4:].set(jnp.asarray(measured["cube_quat_world"], dtype=jnp.float32))
            result["measured_cube_input"] = {
                "source_image": measured["image"],
                "reprojection_px": measured["reprojection_px"],
                "cube_center_from_marker_m": measured["cube_minus_hand_marker_m"],
                "cube_position_sim_m": cube_pos.tolist(),
            }
        data = state.data.replace(qpos=qpos, ctrl=jnp.asarray(recorded_start, dtype=jnp.float32))
        data = mjx.forward(env.mjx_model, data)
        info = dict(state.info)
        info["motor_targets"] = data.ctrl
        info["start_cube_pos"] = data.qpos[-7:-4]
        for key in ("qpos_error_history", "cube_pos_error_history", "cube_ori_error_history"):
            info[key] = jnp.zeros_like(info[key])
        obs = env._get_obs(data, info)
        state = state.replace(data=data, obs=obs, info=info)
        result["recorded_start_test"] = {
            "cube_position_before_step": [float(x) for x in state.data.qpos[-7:-4]],
            "contact_count_before_step": int(state.data._impl.ncon),
        }
        state = jax.jit(env.step)(state, jnp.zeros(16, dtype=jnp.float32))
        result["recorded_start_test"].update({
            "cube_position_after_step": [float(x) for x in state.data.qpos[-7:-4]],
            "contact_count_after_step": int(state.data._impl.ncon),
            "done_after_step": float(state.done),
        })
        if hold_steps:
            hold_start = np.asarray(state.data.qpos[-7:-4], dtype=float)
            hold_step = jax.jit(env.step)
            completed = 0
            for _ in range(hold_steps):
                if bool(state.done):
                    break
                state = hold_step(state, jnp.zeros(16, dtype=jnp.float32))
                completed += 1
            hold_end = np.asarray(state.data.qpos[-7:-4], dtype=float)
            result["fixed_hold_test"] = {
                "requested_steps": hold_steps,
                "completed_steps": completed,
                "cube_start_m": hold_start.tolist(),
                "cube_end_m": hold_end.tolist(),
                "cube_drift_m": float(np.linalg.norm(hold_end - hold_start)),
                "done": float(state.done),
            }
    if "--rollout" in sys.argv:
        policy = Policy(PROJECT / "policy" / "policy_np.npz")
        step = jax.jit(env.step)
        samples = []
        ever_success = False
        steps = 300
        for index in range(steps):
            if bool(state.done):
                break
            action = jnp.asarray(policy(np.asarray(state.obs["state"])), dtype=jnp.float32)
            state = step(state, action)
            ever_success |= bool(float(state.metrics.get("reward/success", 0.0)))
            if index in (0, 19, 39, 59, 79, 99, 149, 199, 249, 299) or bool(state.done):
                samples.append({
                    "step": index + 1,
                    "cube_position": [float(x) for x in state.data.qpos[-7:-4]],
                    "cube_quaternion": [float(x) for x in state.data.qpos[-4:]],
                    "reward": float(state.reward),
                    "done": float(state.done),
                    "success": float(state.metrics.get("reward/success", 0.0)),
                    "orientation_error": float(state.metrics.get("orientation_error", 0.0)),
                })
            if bool(state.done):
                break
        result["steps_run"] = len(samples) and index + 1 or 0
        result["ever_success"] = ever_success
        result["rollout"] = samples
    print(json.dumps(result, indent=2))
    output_name = (
        "sac_sim_contact_candidate_result.json" if contact_candidate
        else "sac_sim_fitted_contact_result.json" if fitted_contact
        else "sac_sim_measured_cube_result.json" if "--measured-cube" in sys.argv
        else "sac_sim_recorded_start_result.json" if "--recorded-start" in sys.argv
        else "sac_sim_rollout_result.json" if "--rollout" in sys.argv
        else "sac_sim_smoke_result.json"
    )
    if real_cube:
        output_name = output_name.replace("_result.json", "_real_cube_result.json")
    if cube_mass_g is not None:
        output_name = output_name.replace("_result.json", f"_{cube_mass_g:g}g_result.json")
    if hold_steps:
        output_name = output_name.replace("_result.json", f"_hold{hold_steps}_result.json")
    if target_face is not None:
        output_name = output_name.replace("_result.json", f"_face{target_face}_result.json")
    if output_file is not None:
        output_name = output_file
    (Path(__file__).resolve().parent / output_name).write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()
