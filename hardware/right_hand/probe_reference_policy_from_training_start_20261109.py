"""Compare saved SAC corrections and zero correction in today's saved model.

Research only: no device modules, no range/contact changes, no hardware handoff.
The slow wrapper interpolates each freshly inferred macro goal, not a saved
action sequence. Its altered timing is explicit and is not training parity.
"""
from pathlib import Path
import importlib.util
import json
import math
import time
import argparse
import numpy as np
import mujoco
from research_rotation_body_hold_20261104 import restore, inspect_for, native, motor
from research_probe_index_handoff_20261109 import ramp
from hand_motor_mapping import ticks_to_sim
from sac_reference_numpy import ReferencePolicy
from sac_forward_geometry import target_quaternion, measure
from saved_policy_observation_provider_20261109 import build_observation

ROOT = Path(__file__).resolve().parent
MODEL = ROOT / 'right_hand_rotation_20261106/middle_clearance_probe/evidence/model.mjb'
START = ROOT / 'right_hand_rotation_20261109/policy_audit/training_start_transfer/sac_cube55_resets_0.npz'
OUT = ROOT / 'right_hand_rotation_20261109/policy_audit/reused_reference_controller'


def observation_type():
    path = ROOT.parent / 'work/Leap_Test/deploy/obs_builder.py'
    spec = importlib.util.spec_from_file_location('saved_observation_builder', path)
    item = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(item)
    return item.ObsBuilder


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--project-collisions', action='store_true')
    parser.add_argument('--preclear-thumb', action='store_true')
    parser.add_argument('--thumb-contact-mm', type=float, default=0.,
                        help='Offline sensitivity only for two explicit thumb/palm pairs; never hardware admission')
    parser.add_argument('--palm-pad-sensitivity-mm', type=float, default=0.,
                        help='Offline bounded pad/palm classification for all four fingertip geoms')
    parser.add_argument('--observation-provider', choices=('training_sensor', 'body_origin'),
                        default='training_sensor', help='body_origin reproduces an incorrect historical diagnostic')
    args = parser.parse_args()
    if args.preclear_thumb and not args.project_collisions:
        parser.error('--preclear-thumb requires --project-collisions')
    if not 0 <= args.thumb_contact_mm <= .5:
        parser.error('Offline thumb/palm sensitivity is bounded to 0..0.5 mm')
    if not 0 <= args.palm_pad_sensitivity_mm <= .5:
        parser.error('Offline pad/palm sensitivity is bounded to 0..0.5 mm')
    if args.palm_pad_sensitivity_mm and (args.project_collisions or args.preclear_thumb or args.thumb_contact_mm):
        parser.error('Pad/palm classification must be an independent comparison')
    began = time.perf_counter()
    output = OUT if not args.project_collisions else OUT.with_name('reused_reference_projected_controller_v2')
    if args.preclear_thumb:
        output = OUT.with_name('reused_reference_preclear_controller')
    if args.thumb_contact_mm:
        if args.project_collisions or args.preclear_thumb:
            parser.error('Contact sensitivity comparison is separate from projected controllers')
        output = OUT.with_name(f'reused_reference_contact_sensitivity_{args.thumb_contact_mm:g}mm')
    if args.palm_pad_sensitivity_mm:
        output = OUT.with_name(f'reused_reference_palm_pad_sensitivity_{args.palm_pad_sensitivity_mm:g}mm')
    if args.observation_provider == 'training_sensor':
        output = output.with_name(output.name + '_training_sensor')
    output.mkdir(parents=True, exist_ok=True)
    policy = ReferencePolicy()
    builder_type = observation_type()
    results = []
    timings = ('original_50ms', 'interpolated_two_ticks_75ms') if not args.project_collisions else ('interpolated_two_ticks_75ms',)
    for timing in timings:
        for correction in ('saved_sac', 'zero_correction'):
            m = mujoco.MjModel.from_binary_path('source.mjb', assets={'source.mjb': MODEL.read_bytes()})
            d = restore(m, np.load(START, allow_pickle=False))
            hand, supports, check = inspect_for(m)
            projector = None
            projection_reports = []
            if args.project_collisions:
                from collision_projection_20261025 import CollisionProjection
                projector = CollisionProjection(m, margin_m=.001, guard_pairs=(
                    ('palm_collision_4', 'th_tip'), ('palm_collision_8', 'th_tip')))
            lo, hi, qlo, qhi = native(m)
            cube = m.geom('cube').id
            body = m.body('cube').id
            palm = m.body('palm').id
            initial = d.qpos[-7:].copy()
            goal = target_quaternion(initial[-4:])
            builder = builder_type()
            previous = d.ctrl.copy()
            for _ in range(3):
                build_observation(builder, m, d, previous, goal, args.observation_provider)
            worst = {'self_mm': 0., 'support_mm': 0., 'cube_mm': 0., 'tracking_rad': 0.}
            raw_goals = []
            states = []
            actions = []
            clock = []
            longest_gap = gap = 0.
            stop = None
            macro_completed = 0
            outer_hold = 0
            seen = []
            start_time = float(d.time)
            prelude = None
            stop_details = None
            pad_force_peak = 0.
            if args.preclear_thumb:
                from collision_projection_20261025 import CollisionProjection
                preprojector = CollisionProjection(m, margin_m=.003, guard_pairs=(
                    ('palm_collision_4', 'th_tip'), ('palm_collision_8', 'th_tip')))
                release, details = preprojector.project(d.qpos[:16], d.qpos[:16], bound_rad=.2)
                prelude = {'projection': details, 'goals': 0, 'completed': False}
                release += d.ctrl - d.qpos[:16]
                release_ticks = motor(release)
                if np.any(release_ticks < lo) or np.any(release_ticks > hi):
                    stop = 'prelude_native_goal_range'
                else:
                    for raw in ramp(motor(previous), release_ticks):
                        d.ctrl[:] = ticks_to_sim(raw)
                        raw_goals.append(raw)
                        prelude['goals'] += 1
                        for _ in range(round(.075 / m.opt.timestep)):
                            mujoco.mj_step(m, d)
                            support, own = check(d)
                            worst['self_mm'] = max(worst['self_mm'], max(own.values(), default=0.))
                            worst['support_mm'] = max(worst['support_mm'], max(support.values(), default=0.))
                            worst['tracking_rad'] = max(worst['tracking_rad'], float(np.max(np.abs(d.qpos[:16] - d.ctrl))))
                            if (worst['self_mm'] > 1e-6 or worst['support_mm'] > .15 or
                                    worst['tracking_rad'] > .18 or np.any(d.qpos[:16] < qlo - 1e-6) or
                                    np.any(d.qpos[:16] > qhi + 1e-6)):
                                stop = 'prelude_original_dynamic_stop'
                                break
                        states.append(d.qpos.copy())
                        clock.append(float(d.time - start_time))
                        if stop:
                            break
                    previous = d.ctrl.copy()
                    prelude['completed'] = stop is None
            for index in range(100):
                if stop:
                    break
                observation = build_observation(builder, m, d, previous, goal, args.observation_provider)
                elapsed = index * .05
                phase = np.array([min(elapsed / 3.6, 1.), float(elapsed >= 3.6)])
                if correction == 'saved_sac':
                    action = policy.action(np.r_[observation, phase])
                else:
                    action = np.zeros(16)
                desired = np.clip(policy.reference(index) + .05 * action, policy.lower, policy.upper)
                executed = np.clip((desired - previous) / .2, -1, 1)
                proposed = np.clip(previous + .2 * executed, policy.lower, policy.upper)
                if projector is not None:
                    # Approximate current servo lag in the proposal only; physics
                    # retains the actual state and the original dynamic checks.
                    lag = d.ctrl - d.qpos[:16]
                    projected, details = projector.project(proposed - lag, d.qpos[:16], bound_rad=.15)
                    projection_reports.append(details)
                    if details['maximum_overlap_mm'] > .000001 or not details['native_ranges_satisfied']:
                        stop = 'collision_projection_failed_without_relaxing_geometry'
                        break
                    proposed = projected + lag
                target_ticks = motor(proposed)
                if np.any(target_ticks < lo) or np.any(target_ticks > hi):
                    stop = 'native_goal_range'
                    break
                if timing == 'interpolated_two_ticks_75ms':
                    subgoals = ramp(motor(previous), target_ticks)
                    targets = [ticks_to_sim(ticks) for ticks in subgoals]
                    duration = .075
                else:
                    targets = [proposed]
                    duration = .05
                for target in targets:
                    d.ctrl[:] = target
                    raw_goals.append(motor(target))
                    for _ in range(round(duration / m.opt.timestep)):
                        mujoco.mj_step(m, d)
                        support, own = check(d)
                        worst['self_mm'] = max(worst['self_mm'], max(own.values(), default=0.))
                        worst['support_mm'] = max(worst['support_mm'], max(support.values(), default=0.))
                        worst['tracking_rad'] = max(worst['tracking_rad'], float(np.max(np.abs(d.qpos[:16] - d.ctrl))))
                        external = False
                        inhand = False
                        for contact_index, contact in enumerate(d.contact):
                            pair_names = {m.geom(int(contact.geom1)).name, m.geom(int(contact.geom2)).name}
                            if (contact.dist < 0 and any(name.startswith('palm_collision_') for name in pair_names)
                                    and pair_names.intersection({'if_tip','mf_tip','rf_tip','th_tip'})):
                                force = np.zeros(6)
                                mujoco.mj_contactForce(m, d, contact_index, force)
                                pad_force_peak = max(pad_force_peak, float(force[0]))
                            if cube not in (int(contact.geom1), int(contact.geom2)):
                                continue
                            other = int(contact.geom2) if int(contact.geom1) == cube else int(contact.geom1)
                            if int(m.geom_bodyid[other]) in hand:
                                inhand |= contact.dist < .0004
                                worst['cube_mm'] = max(worst['cube_mm'], max(0., -float(contact.dist) * 1000))
                            elif other in supports and contact.dist < .0004:
                                external = True
                        gap = 0. if inhand else gap + m.opt.timestep
                        longest_gap = max(longest_gap, gap)
                        named = {'palm_collision_4|th_tip', 'palm_collision_8|th_tip'}
                        if args.palm_pad_sensitivity_mm:
                            named = {key for key in own if
                                     any(name.startswith('palm_collision_') for name in key.split('|'))
                                     and set(key.split('|')).intersection({'if_tip','mf_tip','rf_tip','th_tip'})}
                        other_self = max((value for key,value in own.items() if key not in named), default=0.)
                        thumb_depth = max((value for key,value in own.items() if key in named), default=0.)
                        contact_threshold = max(args.thumb_contact_mm, args.palm_pad_sensitivity_mm)
                        self_stop = (other_self > 1e-6 or thumb_depth > max(1e-6, contact_threshold))
                        if (self_stop or worst['support_mm'] > .15 or worst['cube_mm'] > 1 or
                                worst['tracking_rad'] > .18 or np.any(d.qpos[:16] < qlo - 1e-6) or
                                np.any(d.qpos[:16] > qhi + 1e-6) or external or longest_gap > .25):
                            stop = 'unchanged_dynamic_contact_tracking_range_or_retention_stop'
                            stop_details = {'self_pairs_mm': own, 'support_pairs_mm': support,
                                            'named_thumb_depth_mm': thumb_depth, 'other_self_mm': other_self,
                                            'pad_threshold_mm_diagnostic': contact_threshold,
                                            'self_contact_triggered': self_stop,
                                            'external_cube_support': external, 'longest_hand_gap_s': longest_gap}
                            break
                        if d.time - start_time > 240:
                            stop = 'research_240_simulated_second_limit'
                            break
                    states.append(d.qpos.copy())
                    clock.append(float(d.time - start_time))
                    if stop:
                        break
                if stop:
                    break
                previous = np.asarray(targets[-1]).copy()
                builder.set_last_act(executed)
                actions.append(action)
                macro_completed += 1
                tip, axis, error = [float(v) for v in measure(initial[-4:], d.qpos[-4:])]
                shift = float(np.linalg.norm(d.qpos[-7:-4] - initial[:3]) * 1000)
                eligible = abs(np.degrees(tip) - 90) < 5 and np.degrees(error) < 5 and shift < 30
                outer_hold = outer_hold + 1 if eligible else 0
                seen.append({'macro': index + 1, 'simulated_seconds': float(d.time - start_time),
                             'tip_deg': math.degrees(tip), 'axis_deg': math.degrees(axis),
                             'target_error_deg': math.degrees(error), 'center_shift_mm': shift})
                if macro_completed % 10 == 0:
                    print(json.dumps({'timing': timing, 'correction': correction, **seen[-1]}), flush=True)
            tip, axis, error = [float(v) for v in measure(initial[-4:], d.qpos[-4:])]
            prefix = timing + '_' + correction
            np.savez_compressed(output / (prefix + '.npz'), goals=np.asarray(raw_goals), qpos=np.asarray(states),
                                times=np.asarray(clock), actor_actions=np.asarray(actions))
            result = {'timing': timing, 'correction': correction, 'completed_policy_updates': macro_completed,
                      'low_level_goals': len(raw_goals), 'stop': stop,
                      'simulated_seconds': float(d.time - start_time), 'worst': worst,
                      'tip_deg': math.degrees(tip), 'target_error_deg': math.degrees(error),
                      'center_shift_mm': float(np.linalg.norm(d.qpos[-7:-4] - initial[:3]) * 1000),
                      'task_success_certified': False, 'physical_execution_allowed': False,
                      'source_start_is_synthetic_training_state_not_current_hand_reading': True,
                      'observation_provider': args.observation_provider,
                      'observation_position_contract_corrected': args.observation_provider == 'training_sensor',
                      'unchanged_model_and_limits': True,
                      'slow_wrapper_changes_policy_timing': timing != 'original_50ms',
                      'collision_projection_enabled': projector is not None,
                      'projection_reports': projection_reports,
                      'thumb_preclear': prelude,
                      'stop_details': stop_details,
                      'named_thumb_contact_sensitivity_mm': args.thumb_contact_mm,
                      'original_zero_self_contact_guard_passed': worst['self_mm'] <= 1e-6,
                      'diagnostic_contact_sensitivity_not_physical_admission': max(args.thumb_contact_mm, args.palm_pad_sensitivity_mm) > 0,
                      'palm_pad_contact_sensitivity_mm': args.palm_pad_sensitivity_mm,
                      'palm_pad_normal_force_peak_N_model_only': pad_force_peak,
                      'trajectory': seen}
            results.append(result)
            (output / (prefix + '.json')).write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
            print(json.dumps({k: v for k, v in result.items() if k not in ('trajectory', 'projection_reports')}), flush=True)
            (output / 'review.json').write_text(json.dumps({'status': 'running' if len(results) < len(timings)*2 else 'completed',
                                                        'cases': results, 'devices_opened': 0,
                                                        'hardware_execution_allowed': False,
                                                        'elapsed_seconds': time.perf_counter() - began}, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
