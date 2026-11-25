"""Offline direction check for the first, forward whole-cube flip.

The existing face-up task does not constrain the route. This check makes a
sideways yaw fail even when a front-face color changes. It reads archived
diagnostics only and never imports motor-control code.
"""

import json
import argparse
import sys
from pathlib import Path

import numpy as np
from sac_forward_geometry import measure, METRIC_VERSION


ROOT = Path(__file__).resolve().parent
GREEN = np.asarray([0., -1., 0.])
ORANGE = np.asarray([-1., 0., 0.])


def matrix(quat):
    w, x, y, z = np.asarray(quat, dtype=float)
    n = w*w + x*x + y*y + z*z
    if n < 1e-12:
        raise ValueError('Zero cube quaternion')
    w, x, y, z = np.asarray([w, x, y, z]) / np.sqrt(n)
    return np.asarray([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def motion_from_quats(start, end):
    r0, r1 = matrix(start), matrix(end)
    green0, green1 = r0 @ GREEN, r1 @ GREEN
    orange0, orange1 = r0 @ ORANGE, r1 @ ORANGE
    tip, off_axis, target_error = measure(start, end)
    _, _, reverse_error = measure(start, end, -1)
    return dict(
        metric_version=METRIC_VERSION,
        green_tip_toward_initial_orange_front_deg=float(np.degrees(tip)),
        off_axis_error_deg=float(np.degrees(off_axis)),
        target_error_deg=float(np.degrees(target_error)),
        reverse_target_error_deg=float(np.degrees(reverse_error)),
        green_normal_change_deg=float(np.degrees(np.arccos(
            np.clip(green0 @ green1, -1, 1)))),
    )


def verdict(motion, direction_sign=1):
    if direction_sign not in (-1, 1):
        raise ValueError('direction_sign must be -1 or +1')
    tip = float(motion['green_tip_toward_initial_orange_front_deg'])
    desired_tip = direction_sign * tip
    modern = motion.get('metric_version') == METRIC_VERSION
    deviation = float(motion['off_axis_error_deg'] if modern else
                      abs(motion['orange_front_turn_about_initial_green_axis_deg']))
    normal = float(motion.get('green_normal_change_deg',
                              motion.get('mount_relative_green_normal_change_deg')))
    # This is a deliberately conservative *evaluation* rule, not a motor limit.
    # A 90-degree forward flip should pass 60 degrees along the intended axis.
    forward_progress = 30 <= desired_tip <= 110 and normal >= 30 and deviation <= 25
    goal_error = motion.get('target_error_deg' if direction_sign == 1 else 'reverse_target_error_deg')
    full_forward_flip = bool(modern and 70 <= desired_tip <= 110 and
                             deviation <= 25 and goal_error <= 25)
    wrong_side_turn = deviation >= 15 and deviation > 2 * abs(tip) and normal < 30
    return dict(forward_progress=forward_progress,
                full_forward_flip=full_forward_flip,
                wrong_side_turn=wrong_side_turn,
                forward_tip_deg=tip, desired_direction_tip_deg=desired_tip,
                metric_version=METRIC_VERSION if modern else 'legacy_projected_summary',
                legacy_summary_only=not modern,
                off_axis_error_deg=deviation if modern else None,
                legacy_side_yaw_deg=None if modern else deviation,
                top_normal_change_deg=normal)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--policy-result', default='sac_sim_measured_cube_real_cube_100g_hold20_result.json')
    parser.add_argument('--require-forward', action='store_true',
                        help='Exit 2 unless the selected policy rollout shows a full forward flip in saved samples')
    parser.add_argument('--direction-sign', type=int, choices=(-1, 1), default=1)
    args = parser.parse_args()
    files = [ROOT / 'sac_relative_cube_motion_result.json',
             ROOT / 'sac_recorded_encoder_replay_result.json']
    files += sorted(ROOT.glob('sac_recorded_encoder_replay_offset_*_result.json'))
    files += [ROOT / 'sac_recorded_encoder_replay_stage2_same_session_result.json',
              ROOT / 'sac_recorded_encoder_replay_stage2_offset_1.875_0_0_friction_2_result.json']
    checks = []
    for path in files:
        data = json.loads(path.read_text(encoding='utf-8'))
        checks.append(dict(source=path.name,
                           reference='physical_ID41_relative' if path.name == files[0].name
                           else 'approximate_MuJoCo_replay',
                           **verdict(data['stage_2_to_4'], args.direction_sign)))
    policy_path = ROOT / args.policy_result
    policy_passed = False
    if policy_path.exists():
        policy = json.loads(policy_path.read_text(encoding='utf-8'))
        rollout = policy.get('rollout', [])
        # This archived test had a fixed-goal hold before the policy. Its
        # pre-hold reset quaternion is not the policy-start orientation.
        start = rollout[0]['cube_quaternion'] if rollout else None
        sampled = [verdict(motion_from_quats(start, row['cube_quaternion']),
                           args.direction_sign)
                   for row in rollout] if start is not None else []
        checks.append(dict(source=policy_path.name,
                           reference='approximate_MJX_policy_rollout',
                           face_up_reward_success=bool(policy.get('ever_success')),
                           sampled_forward_progress=any(
                               row['forward_progress'] for row in sampled),
                           sampled_full_forward_flip=any(
                               row['full_forward_flip'] for row in sampled),
                           sample_count=len(sampled),
                           final_sample=sampled[-1] if sampled else None,
                           caveat='Relative to the first saved policy sample (step 1), after the pre-policy hold. Sparse samples and original contact/reset are not calibrated to the October 11 physical trial.'))
        policy_passed = any(row['full_forward_flip'] for row in sampled)
    output = dict(status='offline_first_forward_flip_direction_audit',
                  motor_commands_sent=0,
                  desired_direction_sign=args.direction_sign,
                  metric_version=METRIC_VERSION,
                  thresholds_deg=dict(forward_progress=30, full_flip=[70,110],
                                      max_off_axis=25, max_goal_error=25),
                  checks=checks,
                  interpretation='A front-face color change or face-up reward alone does not establish the requested forward flip. Simulated contact is currently uncalibrated; passing this gate in simulation would not validate transfer to the real hand.')
    path = ROOT / ('sac_forward_flip_gate_v2_result.json' if args.direction_sign == 1
                   else 'sac_reverse_flip_gate_v2_result.json')
    path.write_text(json.dumps(output, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(output, ensure_ascii=False, indent=2))
    if args.require_forward and not policy_passed:
        sys.exit(2)


if __name__ == '__main__':
    main()
