"""Compare corrected native provider with clean training _get_obs at one state."""
from pathlib import Path
import importlib.util
import json
import numpy as np
import mujoco
from mujoco import mjx
import jax
import jax.numpy as jp
from mujoco_playground import registry
from research_rotation_body_hold_20261104 import restore
from sac_forward_geometry import target_quaternion
from sac_reference_numpy import ReferencePolicy
from saved_policy_observation_provider_20261109 import build_observation, sensor_value

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'right_hand_rotation_20261109/policy_audit/environment_transfer'


def main():
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'; cfg.chain_goals = False; cfg.face_onehot_obs = False
    cfg.obs_noise.level = 0.
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model_path = ROOT / 'right_hand_rotation_20261106/middle_clearance_probe/evidence/model.mjb'
    model = mujoco.MjModel.from_binary_path('current.mjb', assets={'current.mjb': model_path.read_bytes()})
    state = np.load(OUT.parent / 'training_start_transfer/sac_cube55_resets_0.npz', allow_pickle=False)
    data = restore(model, state)
    goal = target_quaternion(data.qpos[-4:])
    data.mocap_quat[:] = goal
    mujoco.mj_forward(model, data)
    env._mj_model = model; env._post_init()
    mjx_data = mjx.put_data(model, data, impl='jax')
    path = ROOT.parent / 'work/Leap_Test/deploy/obs_builder.py'
    spec = importlib.util.spec_from_file_location('verified_sensor_obs_builder', path)
    mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    correct, legacy = mod.ObsBuilder(), mod.ObsBuilder()
    info = {
        'rng': jax.random.PRNGKey(0), 'motor_targets': jp.asarray(data.ctrl),
        'last_act': jp.zeros(16), 'qpos_error_history': jp.zeros(48),
        'cube_pos_error_history': jp.zeros(9), 'cube_ori_error_history': jp.zeros(18),
        'pert_dir': jp.zeros(3),
    }
    errors = []
    for _ in range(3):
        expected = np.asarray(env._get_obs(mjx_data, info)['state'])
        actual = build_observation(correct, model, data, data.ctrl, goal)
        previous = build_observation(legacy, model, data, data.ctrl, goal, 'body_origin')
        errors.append(float(np.max(np.abs(expected - actual))))
    palm_sensor = sensor_value(model, data, 'palm_position', 3)
    palm_origin = data.xpos[model.body('palm').id]
    residual = ReferencePolicy()
    actor_difference = float(np.max(np.abs(residual.action(np.r_[actual, 0., 0.]) -
                                           residual.action(np.r_[previous, 0., 0.]))))
    report = {
        'status': 'passed' if max(errors) < 1e-5 else 'failed',
        'comparison': 'Original inherited _get_obs, observation noise disabled, identical current model state',
        'consecutive_history_builds': 3, 'max_observation_error_by_build': errors,
        'palm_sensor_position_m': palm_sensor.tolist(), 'palm_body_origin_m': palm_origin.tolist(),
        'wrong_position_offset_m': (palm_origin - palm_sensor).tolist(),
        'wrong_position_offset_mm': float(np.linalg.norm(palm_origin - palm_sensor) * 1000),
        'legacy_max_observation_error': float(np.max(np.abs(expected - previous))),
        'actor_action_max_difference_at_this_start': actor_difference,
        'new_provider_uses_training_sensor_for_palm_cube_and_orientation': True,
        'not_full_dynamic_parity_or_physical_calibration': True,
        'devices_opened': 0, 'hardware_execution_allowed': False,
    }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / 'observation_provider_verification.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report), flush=True)
    if report['status'] != 'passed':
        raise AssertionError(report)


if __name__ == '__main__':
    main()
