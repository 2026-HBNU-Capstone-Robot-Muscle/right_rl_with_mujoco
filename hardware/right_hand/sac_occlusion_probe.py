"""Offline test: freeze camera-derived cube observation during a simulated turn."""

import json
import sys
from pathlib import Path

import jax
import jax.numpy as jnp
import mujoco
import numpy as np
from mujoco import mjx
from mujoco_playground import registry

from sac_cube_model import set_cube_physics


ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent / "work" / "Leap_Test"
sys.path.insert(0, str(PROJECT / "deploy"))
from policy_np import Policy


def run_one(env, policy, step, hidden_steps, hide_start=20, max_steps=300):
    state = env.reset(jax.random.PRNGKey(0))
    frozen_cube = None
    samples = []
    ever_success = False
    hidden_actions = 0
    for i in range(max_steps):
        if bool(state.done):
            break
        observation = np.asarray(state.obs["state"])
        if i == hide_start:
            frozen_cube = observation.copy()
        if hidden_steps and hide_start <= i < hide_start + hidden_steps:
            observation = observation.copy()
            # 16 joints + 3x16 joint-error history = 64. Camera supplies
            # the next 3x3 cube-position and 3x6 cube-orientation histories;
            # the final scalar orientation error also comes from cube pose.
            observation[64:91] = frozen_cube[64:91]
            observation[107] = frozen_cube[107]
            hidden_actions += 1
        action = jnp.asarray(policy(observation), dtype=jnp.float32)
        state = step(state, action)
        success = bool(float(state.metrics.get("reward/success", 0)))
        ever_success |= success
        if i in (0, 19, 29, 39, 59, 79, 99, 149, 199, 249, 299) or bool(state.done):
            samples.append({
                "step": i + 1,
                "cube_position_m": np.asarray(state.data.qpos[-7:-4]).tolist(),
                "success": success,
                "done": bool(state.done),
                "face_error_rad": float(state.metrics.get("face_error", np.nan)),
            })
    return {"requested_hidden_steps": hidden_steps, "hidden_actions": hidden_actions,
            "steps_run": i + 1, "ever_success": ever_success,
            "final": samples[-1], "samples": samples}


def main():
    cfg = registry.get_default_config("LeapCubeFaceUp")
    cfg.impl = "jax"
    env = registry.load("LeapCubeFaceUp", config=cfg)
    set_cube_physics(env.mj_model, side_m=.0545, mass_kg=.100)
    env._mjx_model = mjx.put_model(env.mj_model, impl="jax")
    policy = Policy(PROJECT / "policy" / "policy_np.npz")
    first = env.reset(jax.random.PRNGKey(0))
    assert np.asarray(first.obs["state"]).shape == (108,), "Observation layout changed"
    assert cfg.history_len == 3, "History layout changed"
    step = jax.jit(env.step)
    results = []
    for duration in (0, 10, 20, 40, 80):
        row = run_one(env, policy, step, duration)
        results.append(row)
        print(f"Hidden {duration} steps: success={row['ever_success']} "
              f"steps={row['steps_run']}", flush=True)
    report = {
        "status": "offline_observation_ablation", "hardware_authorized": False,
        "start": "default_simulator_reset; not measured stage-0 physical placement",
        "cube_side_m": .0545, "cube_mass_kg": .100,
        "hide_start_step": 20, "control_timestep_s": float(cfg.ctrl_dt),
        "frozen_observation_indices": ["64:91", 107],
        "target_quaternion": np.asarray(first.data.mocap_quat[0]).tolist(),
        "cases": results,
        "scope": "Optimistic test of holding the last camera cube pose; actor still sees fresh joints and actions. No online vision or physical pose validation.",
    }
    (ROOT / "sac_occlusion_probe_result.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
