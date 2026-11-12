"""Offline, diagnostic closed-loop policy probes in the current saved model.

Strict integration probes preserve the two-tick goal guard. Raw-semantics
probes omit that one admission guard only to diagnose the old controller;
they retain native/tracking/contact stops and cannot approve hardware motion.
No servo, camera, ROS, or hardware controller is imported.
"""
from pathlib import Path
import importlib.util
import json
import hashlib
import math
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from research_rotation_body_hold_20261104 import restore, inspect_for, native, motor
from hand_motor_mapping import SIM_JOINT_NAMES, ticks_to_sim, TICK
from sac_reference_numpy import ReferencePolicy
from saved_policy_observation_provider_20261109 import build_observation

ROOT = Path(__file__).resolve().parent
WORK = ROOT.parent / 'work/Leap_Test'
MODEL = ROOT / 'right_hand_rotation_20261106/middle_clearance_probe/evidence/model.mjb'
STATES = ROOT / 'right_hand_rotation_20261109/middle_probe_reconstruction_linux'
OUT = ROOT / 'right_hand_rotation_20261109/policy_audit/current_model_probe_training_sensor'


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    item = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(item)
    return item


def quat(B):
    q = Rotation.from_matrix(B).as_quat()
    return np.r_[q[3], q[:3]]


def goal_for_face(B, up):
    # Original face index 2 is local +X. Align its measured normal to gravity-up.
    normal = B[:, 0]
    axis = np.cross(normal, up)
    length = np.linalg.norm(axis)
    angle = math.acos(float(np.clip(normal @ up, -1, 1)))
    if length < 1e-8:
        return B.copy()
    return Rotation.from_rotvec(axis / length * angle).as_matrix() @ B


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    policy = load_module('diagnostic_saved_actor', WORK / 'deploy/policy_np.py').Policy(WORK / 'policy/policy_np.npz')
    builder_type = load_module('diagnostic_saved_observations', WORK / 'deploy/obs_builder.py').ObsBuilder
    reference = ReferencePolicy()
    cases = []
    for checkpoint in (927, 1042):
        state_path = STATES / f'checkpoint_{checkpoint}.npz'
        z = np.load(state_path, allow_pickle=False)
        for kind in ('original_108', 'reference_sac_110'):
            for mode in ('strict_two_tick_admission', 'raw_old_50ms_diagnostic_only'):
                m = mujoco.MjModel.from_binary_path('source.mjb', assets={'source.mjb': MODEL.read_bytes()})
                if tuple(m.joint(i).name for i in range(16)) != tuple(SIM_JOINT_NAMES):
                    raise RuntimeError('Model joint-name order mismatch')
                d = restore(m, z)
                builder = builder_type()
                hand, supports, check = inspect_for(m)
                low, high, qlow, qhigh = native(m)
                body = m.body('cube').id
                cube = m.geom('cube').id
                palm = m.body('palm').id
                up = np.asarray(z['up'])
                B0 = d.xmat[body].reshape(3, 3).copy()
                goal = quat(goal_for_face(B0, up))
                previous = d.ctrl.copy()
                builder.fill_ori_history(quat(B0), goal)
                snapshots = []
                stop = None
                first = None
                worst = {'tracking_rad': 0., 'self_mm': 0., 'support_mm': 0., 'cube_mm': 0.}
                completed = 0
                for step in range(100):
                    B = d.xmat[body].reshape(3, 3)
                    obs = build_observation(builder, m, d, previous, goal)
                    if kind == 'original_108':
                        action = policy(obs)
                        lower = np.array([-.314, -1.047, -.2, -.2] * 3 + [-.349, -.47, -1.2, -1.34])
                        upper = np.array([2.23, 1.047, 1.885, 2.042] * 3 + [2.094, 2.443, 1.9, 1.88])
                        proposed = np.clip(previous + .2 * action, lower, upper)
                    else:
                        elapsed = step * .05
                        augmented = np.r_[obs, min(elapsed / 3.6, 1.), float(elapsed >= 3.6)]
                        proposed, action = reference.command(augmented, previous, step)
                    ticks = motor(proposed)
                    delta = np.abs(ticks - motor(previous))
                    if first is None:
                        first = {'maximum_goal_step_ticks': int(delta.max()),
                                 'maximum_action_abs': float(np.max(np.abs(action))),
                                 'joints_over_two_ticks': int(np.count_nonzero(delta > 2)),
                                 'proposed_goal_ticks': ticks.tolist()}
                    snapshots.append({'step': step, 'observation': obs.tolist(), 'action': action.tolist(),
                                      'proposed_goal_ticks': ticks.tolist(), 'qpos': d.qpos.tolist()})
                    if np.any(ticks < low) or np.any(ticks > high):
                        stop = 'proposed_goal_outside_current_native_range'
                        break
                    if mode == 'strict_two_tick_admission' and delta.max() > 2:
                        stop = 'old_policy_goal_step_exceeds_current_two_tick_admission'
                        break
                    d.ctrl[:] = proposed
                    builder.set_last_act(action)
                    for substep in range(50):
                        mujoco.mj_step(m, d)
                        support, own = check(d)
                        worst['self_mm'] = max(worst['self_mm'], max(own.values(), default=0.))
                        worst['support_mm'] = max(worst['support_mm'], max(support.values(), default=0.))
                        worst['tracking_rad'] = max(worst['tracking_rad'], float(np.max(np.abs(d.qpos[:16] - d.ctrl))))
                        for c in d.contact:
                            if cube in (int(c.geom1), int(c.geom2)):
                                other = int(c.geom2) if int(c.geom1) == cube else int(c.geom1)
                                if int(m.geom_bodyid[other]) in hand:
                                    worst['cube_mm'] = max(worst['cube_mm'], max(0., -float(c.dist) * 1000))
                        if (worst['self_mm'] > 1e-6 or worst['support_mm'] > .15 or
                                worst['cube_mm'] > 1 or worst['tracking_rad'] > .18 or
                                np.any(d.qpos[:16] < qlow - 1e-6) or np.any(d.qpos[:16] > qhigh + 1e-6)):
                            stop = 'current_dynamic_contact_tracking_or_range_stop'
                            break
                    if stop:
                        break
                    previous = proposed.copy()
                    completed += 1
                B = d.xmat[body].reshape(3, 3)
                row = {'checkpoint': checkpoint, 'policy': kind, 'mode': mode,
                       'first_proposal': first, 'completed_50ms_goals': completed, 'stop': stop,
                       'worst': worst,
                       'target_local_plus_x_face_up_error_deg': float(np.degrees(np.arccos(np.clip(B[:, 0] @ up, -1, 1)))),
                       'physical_execution_allowed': False,
                       'observation_provider': 'training_sensor',
                       'trained_task_frame_compatibility_verified': False,
                       'reference_policy_is_fixed_direction_not_arbitrary_goal_trained': kind == 'reference_sac_110'}
                cases.append(row)
                prefix = f'cp{checkpoint}_{kind}_{mode}'
                (OUT / (prefix + '.json')).write_text(json.dumps({'summary': row, 'trace': snapshots}, indent=2) + '\n', encoding='utf-8')
                print(json.dumps(row), flush=True)
    report = {'status': 'current_model_diagnostic_policy_probes_complete',
              'model_sha256': hashlib.sha256(MODEL.read_bytes()).hexdigest(),
              'cases': cases, 'observation_source': 'exact saved simulator state, not live camera',
               'observation_position_contract_corrected': True,
              'task_frame_assumptions_are_diagnostic_not_calibrated_hardware': True,
              'raw_50ms_mode_is_not_current_two_tick_motion_admission': True,
              'devices_opened': 0, 'hardware_execution_allowed': False,
              'policy_retraining_performed': False}
    (OUT / 'review.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
