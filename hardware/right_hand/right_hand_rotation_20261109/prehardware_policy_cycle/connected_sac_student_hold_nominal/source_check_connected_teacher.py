"""Fresh uninterrupted SAC prefix and saved model-teacher suffix; no devices.

The optional return probe reverses recorded joint commands as a physics baseline.
It is neither a learned next-turn controller nor a hardware execution file.
"""
import argparse
import json
import time
import numpy as np
import mujoco
from native_cycle import OUT, assets, restore, Guard, motor, ticks_to_sim, BOUND, save_json, sha
from export_and_evaluate import Actor, CASES
from probe_reference_policy_from_training_start_20261109 import observation_type
from saved_policy_observation_provider_20261109 import build_observation
from cube_task_frame_20261105 import upright_yaw_target, orientation_error_deg
from inspect_endpoints import inspect
from upper_layer_cylinder_clearance_20261105 import precompute, gaps
from upper_layer_separation_certificate_20261105 import SweepCertificate
from research_rotation_body_hold_20261104 import angle


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--return-probe', action='store_true')
    parser.add_argument('--case',choices=('nominal','x_plus05','friction025'),default='nominal')
    parser.add_argument('--teacher',default='next_model_feedback_5_all_fingers')
    parser.add_argument('--output',default='connected_sac_and_teacher')
    parser.add_argument('--student-run',help='Use a learned correction actor rather than replay teacher joint goals')
    parser.add_argument('--acquire-samples',type=int,default=8)
    args = parser.parse_args()
    folder = OUT / args.output
    folder.mkdir(exist_ok=True)
    if (folder/'review.json').exists():raise RuntimeError('Preserve completed connected trial')
    from pathlib import Path
    (folder/'source_check_connected_teacher.py').write_bytes(Path(__file__).read_bytes())
    m, z, reference, _, provenance = assets('active')
    d = restore(m, z)
    guard = Guard(m)
    parameters=CASES[args.case]
    d.qpos[-7:-4]+=np.array([parameters.get('dx_mm',0.),parameters.get('dy_mm',0.),0.])/1000
    if 'friction' in parameters:
        for gid in range(m.ngeom):
            if int(m.geom_bodyid[gid]) in guard.hand or gid==m.geom('cube').id:m.geom_friction[gid,0]=parameters['friction']
    if parameters:mujoco.mj_forward(m,d)
    cube = m.geom('cube').id
    cache = precompute(m, guard.hand)
    target = upright_yaw_target(z['initial_cube_rotation'], z['up'], 90)
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, target.ravel())
    selected = OUT / 'sac_current_contract_v4_20k/export_000000010256'
    manifest = json.loads((selected / 'manifest.json').read_text())
    if sha(selected / 'actor.npz') != manifest['actor_sha256']:
        raise RuntimeError('Actor hash changed')
    actor = Actor(selected / 'actor.npz')
    builder = observation_type()()
    for _ in range(3):
        build_observation(builder, m, d, d.ctrl, quat)
    prefix_folder=selected/('evaluation_active_certified' if args.case=='nominal' else 'evaluation_active_'+args.case)
    prefix = dict(np.load(prefix_folder / 'closed_loop_trace.npz', allow_pickle=False))
    teacher_folder = OUT / args.teacher
    teacher = None if args.student_run else dict(np.load(teacher_folder / 'research_goals.npz', allow_pickle=False))
    station = np.load(OUT / 'baseline_archived_zero/end_state.npz', allow_pickle=False)['qpos'][-7:-4]
    stop = None
    all_goals = []
    trace = []
    began = time.perf_counter()
    certificate = SweepCertificate(m, guard.hand, .3)
    tail = dict(samples=0, lower=0, bad=0, gap=float('inf'), yaw_drift=0., center_drift=0.)
    tail_B = tail_C = None

    def sample():
        B = d.geom_xmat[cube].reshape(3, 3)
        C = d.geom_xpos[cube]
        force = {}
        for ci, contact in enumerate(d.contact):
            a, b = int(contact.geom1), int(contact.geom2)
            if cube not in (a, b):
                continue
            other = b if a == cube else a
            name = m.geom(other).name
            if name in ('if_tip', 'th_tip') and -(contact.pos-C) @ B[:, 1] * 1000 < 55/6:
                f = np.zeros(6)
                mujoco.mj_contactForce(m, d, ci, f)
                force[name] = force.get(name, 0.) + float(f[0])
        return dict(error_deg=orientation_error_deg(B, target),
                    station_error_mm=float(np.linalg.norm(C-station)*1000),
                    upper_gap_mm=min(v for _, v in gaps(m, d, C, B, cache, True)),
                    lower=all(force.get(name, 0.) > .2 for name in ('if_tip', 'th_tip')),
                    lower_forces=force)

    def execute(goal, phase, certify=False):
        nonlocal stop, tail_B, tail_C
        if np.max(np.abs(goal-motor(d.ctrl))) > 2:
            stop = ['two_tick_command_spacing']
            return
        if np.any(goal < guard.lo) or np.any(goal > guard.hi):
            stop = ['native_goal_range']
            return
        d.ctrl[:] = ticks_to_sim(goal)
        all_goals.append(goal.copy())
        for _ in range(75):
            mujoco.mj_step(m, d)
            stop = guard.check(d)
            if np.linalg.norm(d.qpos[-7:-4]-z['initial_cube_center']) > .08:
                stop = list(stop) + ['cube_translation_over80mm']
            if certify:
                B = d.geom_xmat[cube].reshape(3, 3)
                C = d.geom_xpos[cube]
                if tail_B is None:
                    tail_B = B.copy()
                    tail_C = C.copy()
                state = sample()
                tail['samples'] += 1
                tail['lower'] += state['lower']
                certified, bad = certificate.query(d, C, B)
                tail['bad'] += bool(bad)
                tail['gap'] = min(tail['gap'], certified)
                tail['yaw_drift'] = max(tail['yaw_drift'], abs(angle(B, tail_B, z['up'])))
                tail['center_drift'] = max(tail['center_drift'], float(np.linalg.norm(C-tail_C)*1000))
            if stop:
                break
        if len(all_goals) % 100 == 0 or certify or stop:
            trace.append(dict(goal=len(all_goals), phase=phase, **sample()))

    mismatches = 0
    goals = reference['goals']
    for i in range(len(prefix['goals'])):
        obs = build_observation(builder, m, d, d.ctrl, quat)
        vector = np.r_[obs, min(i/max(1, len(goals)-1), 1.), float(i >= len(goals)-134)]
        action = actor.action(vector)
        previous = motor(d.ctrl)
        proposal = motor(ticks_to_sim(goals[min(i, len(goals)-1)]) + manifest['residual_rad']*action)
        goal = previous + np.clip(proposal-previous, -2, 2)
        mismatches += not np.array_equal(goal, prefix['goals'][i])
        old_ctrl = d.ctrl.copy()
        execute(goal, 'live_SAC_prefix')
        builder.set_last_act(np.clip((d.ctrl-old_ctrl)/.2, -1, 1))
        if (i+1) % 1000 == 0:
            print(json.dumps(dict(prefix_goals=i+1, mismatches=mismatches, stop=stop)), flush=True)
        if stop:
            break
    original_seam = dict(np.load(prefix_folder / 'end_state.npz', allow_pickle=False))
    seam = {key: float(np.max(np.abs(getattr(d, key)-original_seam[key]))) if original_seam[key].size else 0.
            for key in BOUND}
    if mismatches or max(seam.values()) > 1e-8:
        stop = ['fresh_SAC_prefix_does_not_reproduce_source_state']
    acquisition_step=None
    if args.student_run:
        student=Actor(OUT/args.student_run/'actor.npz');student_builder=observation_type()()
        for _ in range(3):build_observation(student_builder,m,d,d.ctrl,quat)
        consecutive=0;latched=False
        from cube_task_frame_20261105 import face_up_error_deg
        for i in range(256):
            if stop:break
            obs=np.r_[build_observation(student_builder,m,d,d.ctrl,quat),1.,min(consecutive/54,1.)]
            action=np.zeros(16) if latched else student.action(obs)
            previous_ctrl=d.ctrl.copy();goal=motor(d.ctrl+2*(2*np.pi/4096)*action)
            execute(goal,'learned_correction_then_fixed_position_hold',certify=i>=202)
            student_builder.set_last_act(np.clip((d.ctrl-previous_ctrl)/.2,-1,1))
            current=sample();B=d.geom_xmat[cube].reshape(3,3)
            qualifies=current['error_deg']<1.5 and current['station_error_mm']<2 and current['upper_gap_mm']>.3 and face_up_error_deg(B,5,z['up'])<5
            consecutive=consecutive+1 if qualifies else 0
            if not latched and consecutive>=args.acquire_samples:latched=True;acquisition_step=i+1
    else:
        for i, goal in enumerate(teacher['goals']):
            if stop:break
            execute(goal, 'saved_model_teacher_not_SAC', certify=i >= len(teacher['goals'])-54)
    endpoint = inspect(m, d, z['up'], z['initial_cube_rotation'])
    state = sample()
    why = []
    if stop: why.append('dynamic_or_seam_guard')
    if args.student_run and acquisition_step is None: why.append('fine_acquisition_not_reached')
    if state['error_deg'] >= 5: why.append('quarter_orientation')
    if abs(endpoint['yaw_deg']-90) >= 5: why.append('quarter_yaw')
    if endpoint['up_tilt_deg'] >= 5: why.append('tilt')
    if state['station_error_mm'] >= 2: why.append('common_grip_station')
    if tail['samples'] != 4050: why.append('hold_not_completed')
    if tail['lower']/max(1, tail['samples']) <= .8: why.append('lower_grip')
    if tail['bad'] or tail['gap'] <= .3: why.append('upper_space_certificate')
    if tail['yaw_drift'] >= 3 or tail['center_drift'] >= 2: why.append('hold_drift')
    np.savez_compressed(folder / 'first_quarter_end_state.npz', **{key:getattr(d, key) for key in BOUND},
                        time=d.time, up=z['up'], initial_cube_rotation=z['initial_cube_rotation'],
                        initial_cube_center=z['initial_cube_center'])
    forward_goals = np.array(all_goals)
    report = dict(status='fresh_connected_SAC_prefix_and_offline_teacher_suffix', provenance=provenance,
                  case=args.case,hypothetical_case_parameters=parameters,
                  saved_teacher_folder=None if args.student_run else args.teacher,teacher_is_privileged_model_control=not bool(args.student_run),
                  prefix_live_actor_calls=len(prefix['goals']), prefix_goal_mismatches=mismatches,
                  prefix_seam_max_abs=seam, goals_completed=len(all_goals),
                  source_teacher_sha256=None if args.student_run else sha(teacher_folder / 'research_goals.npz'),
                  first_rotation_and_hold_passed=not why, reasons=why, endpoint=endpoint,
                  common_station_error_mm=state['station_error_mm'], final_hold=tail,
                  separation_certificate=certificate.metadata(), worst=dict(guard.worst),
                  no_state_reset_after_initial_placement=True, suffix_is_saved_model_teacher_not_SAC=not bool(args.student_run),
                  student_run=args.student_run,student_actor_sha256=sha(OUT/args.student_run/'actor.npz') if args.student_run else None,
                  student_is_behavior_cloned_correction_not_SAC=bool(args.student_run),
                  acquisition_step=acquisition_step,acquisition_qualifying_samples=args.acquire_samples,
                  hold_uses_fixed_position_command=bool(args.student_run),
                  next_rotation_passed=False, hardware_execution_allowed=False, devices_opened=0)
    if args.return_probe and not stop:
        # A deliberately separate joint-path reversibility comparison, not SAC.
        target[:] = upright_yaw_target(z['initial_cube_rotation'], z['up'], 0)
        return_start = len(all_goals)
        candidate = None
        for i, goal in enumerate(forward_goals[-2::-1]):
            execute(goal, 'reversed_joint_path_baseline_not_SAC')
            if stop:
                break
            state = sample()
            if state['error_deg'] < 5 and state['station_error_mm'] < 2 and state['lower'] and state['upper_gap_mm'] > .3:
                candidate = dict(index=i, **state)
                break
        if candidate is not None and not stop:
            tail_B = tail_C = None
            certificate = SweepCertificate(m, guard.hand, .3)
            tail = dict(samples=0, lower=0, bad=0, gap=float('inf'), yaw_drift=0., center_drift=0.)
            goal = motor(d.ctrl)
            for i in range(134):
                execute(goal, 'return_constant_goal_hold_not_SAC', certify=i >= 80)
                if stop:
                    break
        report['reverse_joint_baseline'] = dict(goals_completed=len(all_goals)-return_start, stop=stop,
                                                reached_target_and_grip_candidate=candidate,
                                                endpoint=inspect(m, d, z['up'], z['initial_cube_rotation']),
                                                final=sample(), hold=tail if candidate is not None else None,
                                                is_learned_policy=False, task_success_certified=False)
    np.savez_compressed(folder / 'research_trace.npz', goals=all_goals, first_segment_goals=forward_goals)
    report['trace'] = trace
    report['elapsed_seconds'] = time.perf_counter()-began
    save_json(folder / 'review.json', report)
    print(json.dumps({key:value for key,value in report.items() if key not in ('trace','provenance')}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
