"""Receding-horizon native model teacher for entry and next-turn bridges.

Branches are hypothetical model predictions. One persistent actual model state
executes selected controls without resetting the cube or softening any guard.
This is neither SAC training nor a physical robot controller.
"""
from pathlib import Path
import argparse
import json
import time
from collections import Counter, deque
import numpy as np
import mujoco
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation
from native_cycle import OUT, assets, restore, Guard, BOUND, motor, ticks_to_sim, SIM_TO_MOTOR_IDS, save_json, sha
from whole_hand_support import sample_model
from convex_pair_gap import ConvexPairGap
from cube_task_frame_20261105 import upright_yaw_target, orientation_error_deg, face_up_error_deg
from upper_layer_cylinder_clearance_20261105 import precompute, gaps
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--mode',choices=('entry','next'),required=True)
    parser.add_argument('--source-index',type=int,default=1700);parser.add_argument('--increment',type=float,default=90.)
    parser.add_argument('--steps',type=int,default=400);parser.add_argument('--horizon',type=int,default=12)
    parser.add_argument('--commit',type=int,default=4);parser.add_argument('--margin-mm',type=float,default=1.)
    parser.add_argument('--transport-first',action='store_true');parser.add_argument('--start-state')
    parser.add_argument('--station-mode',choices=('reference','canonical'),default='canonical')
    parser.add_argument('--center-weight',type=float,default=.4);parser.add_argument('--joint-weight',type=float)
    parser.add_argument('--hybrid-centering',action='store_true')
    parser.add_argument('--ring14-reserve-deg',type=float,default=0.)
    parser.add_argument('--contact-pressure-target-N',type=float,default=0.)
    parser.add_argument('--contact-pressure-floor-N',type=float,default=0.)
    parser.add_argument('--include-ring-thumb-pairs',action='store_true')
    parser.add_argument('--include-middle-thumb-pair',action='store_true')
    parser.add_argument('--contact-fingers',default='rf,th')
    parser.add_argument('--require-loaded-hold',action='store_true')
    parser.add_argument('--contact-pressure-floor-weight',type=float,default=8.)
    parser.add_argument('--retain-loaded-contacts',action='store_true')
    parser.add_argument('--sliding-friction',type=float,help='Explicit offline friction hypothesis; geometry and guards remain unchanged')
    parser.add_argument('--include-middle-self-pair',action='store_true')
    parser.add_argument('--ID6-upper-reserve-ticks',type=int,default=0,help='Keep commands inside the unchanged ID6 native upper limit')
    parser.add_argument('--entry-command-tolerance-ticks',type=float,help='Require reference hand commands before entry hold; does not relax native guards')
    parser.add_argument('--goal-reference');parser.add_argument('--tag',required=True);args=parser.parse_args()
    if not args.tag.replace('_','').isalnum() or not 1<=args.commit<=args.horizon<=24 or not 1<=args.steps<=800 or not 0<=args.margin_mm<=3:raise ValueError('Bounded research settings required')
    if not .1<=args.center_weight<=2 or (args.joint_weight is not None and not 0<=args.joint_weight<=.2):raise ValueError('Bounded objective weights required')
    if not 0<=args.ID6_upper_reserve_ticks<=4:raise ValueError('Bounded ID6 command reserve required')
    if args.entry_command_tolerance_ticks is not None and (args.mode!='entry' or not 0<=args.entry_command_tolerance_ticks<=32):raise ValueError('Bounded entry-only hand alignment tolerance required')
    if not 0<=args.ring14_reserve_deg<=15 or not 0<=args.contact_pressure_target_N<=5:raise ValueError('Bounded relief objectives required')
    if not 0<=args.contact_pressure_floor_N<=2 or (args.contact_pressure_target_N and args.contact_pressure_floor_N>args.contact_pressure_target_N):raise ValueError('Bounded contact-floor objective required')
    contact_fingers=args.contact_fingers.split(',')
    if not contact_fingers or len(set(contact_fingers))!=len(contact_fingers) or any(f not in ('if','mf','rf','th') for f in contact_fingers):raise ValueError('Known distinct contact fingers required')
    if not 1<=args.contact_pressure_floor_weight<=100:raise ValueError('Bounded contact-planning weight required')
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/args.tag
    if folder.exists():raise RuntimeError('Preserve previous bridge study')
    model,initial,_,_,provenance=assets('active_ccd50');cube=model.geom('cube').id
    original_friction=model.geom_friction.copy()
    if args.sliding_friction is not None:
        if not .2<=args.sliding_friction<=.4:raise ValueError('Bounded offline sliding-friction hypothesis required')
        model.geom_friction[:,0]=args.sliding_friction
        assert np.array_equal(model.geom_friction[:,1:],original_friction[:,1:])
    reference_path=base/'RF_clear_work_grip_joined_suffix/end_state.npz'
    if args.goal_reference:
        reference_path=(OUT/args.goal_reference).resolve()
        if not reference_path.is_relative_to(OUT.resolve()):raise ValueError('Goal reference must be local research data')
    reference=dict(np.load(reference_path,allow_pickle=False))
    if args.mode=='entry':
        source=base/'early_CCD50_states/state_pool.npz';pool=np.load(source,allow_pickle=False)
        index=np.flatnonzero(pool['indices']==args.source_index)
        if len(index)!=1:raise ValueError('Unknown actual parent checkpoint')
        z=dict(initial);z.update({key:pool[key][index[0]].copy() for key in BOUND});z['time']=float(pool['time'][index[0]])
        global_yaw=90.;orientation_tolerance=5.
    else:
        if not 1<=args.increment<=90:raise ValueError('Bounded next-turn increment required')
        source=reference_path;z=dict(reference);global_yaw=90.+args.increment;orientation_tolerance=min(5.,args.increment*.2)
    if args.start_state:
        source=(OUT/args.start_state).resolve()
        if not source.is_relative_to(OUT.resolve()):raise ValueError('Start state must be local research data')
        z=dict(np.load(source,allow_pickle=False))
    data=restore(model,z);guard=Guard(model);start_ctrl=data.ctrl.copy();start_center=data.geom_xpos[cube].copy();start_rotation=data.geom_xmat[cube].reshape(3,3).copy()
    command_lo=np.ceil(guard.lo).astype(int);command_hi=np.floor(guard.hi).astype(int)
    command_hi[5]-=args.ID6_upper_reserve_ticks
    if args.ID6_upper_reserve_ticks and np.any(motor(data.ctrl)>command_hi):raise ValueError('Start command must already respect the requested ID6 reserve')
    target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],global_yaw)
    reference_data=restore(model,reference);reference_center=reference_data.geom_xpos[cube].copy()
    canonical_station=np.load(OUT/'baseline_archived_zero/end_state.npz',allow_pickle=False)['qpos'][-7:-4]
    station=canonical_station.copy() if args.station_mode=='canonical' else reference_center.copy()
    geometry=precompute(model,guard.hand);sid=np.array(SIM_TO_MOTOR_IDS)-1
    pair_names=[('mf_tip',f'palm_collision_{i}') for i in (1,4,6,8)]
    pair_names += [('mf_tip','rf_md_collision_5'),('cadtip_mf_rf_mf_md_0','cadtip_mf_rf_rf_md_0'),('cadtip_mf_rf_mf_px_0','cadtip_mf_rf_rf_md_0')]
    if args.include_ring_thumb_pairs:pair_names += [('rf_tip','palm_collision_4'),('rf_tip','palm_collision_8'),('th_tip','palm_collision_2')]
    if args.include_middle_thumb_pair:pair_names += [('cadtip_mf_th_mf_ds_0','cadtip_mf_th_th_ds_0')]
    if args.include_middle_self_pair:pair_names += [('mf_bs_collision_1','mf_tip')]
    pairs=[(model.geom(a).id,model.geom(b).id) for a,b in pair_names]
    from planning_pair_gap import PlanningPairGap
    separation=PlanningPairGap(model,sorted({geom for pair in pairs for geom in pair}))
    finger_ids={f:{gid for gid in range(model.ngeom) if (model.body(int(model.geom_bodyid[gid])).name or '').startswith(f+'_')} for f in contact_fingers}
    def contact_forces(state):
        force={f:0. for f in contact_fingers};wrench=np.zeros(6)
        for cidx,c in enumerate(state.contact):
            a,b=int(c.geom1),int(c.geom2)
            if cube not in (a,b):continue
            other=b if a==cube else a;mujoco.mj_contactForce(model,state,cidx,wrench)
            for f,ids in finger_ids.items():
                if other in ids:force[f]+=max(0.,float(wrench[0]))
        return force
    rejections=Counter();planning_calls=0;rows=[];trace=[];trajectory=[data.qpos.copy()];began=time.perf_counter()
    transport_complete=not args.transport_first;transport_rotation=start_rotation.copy();phase_switches=[]
    folder.mkdir();(folder/'source_plan_clear_work_grip_bridge.py').write_bytes(Path(__file__).read_bytes())
    np.savez_compressed(folder/'start_state.npz',**{key:getattr(data,key).copy() for key in BOUND},time=data.time,
        up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])

    def evidence(state):
        B=state.geom_xmat[cube].reshape(3,3);C=state.geom_xpos[cube]
        rotation=np.degrees(Rotation.from_matrix((target if transport_complete else transport_rotation)@B.T).as_rotvec())
        load=sample_model(model,state,guard.hand,cube)
        pair_gaps=np.array([separation.gap(state,a,b)*1000 for a,b in pairs])
        tracking=np.max(abs(state.qpos[:16]-state.ctrl))
        upper=min(value for _,value in gaps(model,state,C,B,geometry,True))
        error=orientation_error_deg(B,target)
        residual=np.r_[rotation,(C-station)*1000*(args.center_weight if transport_complete else 1.2),
            np.degrees(state.ctrl-reference['ctrl'])*(args.joint_weight if args.joint_weight is not None else (.025 if args.mode=='next' else .06)),
            np.maximum(0.,args.margin_mm-pair_gaps)*20.,
            max(0.,tracking-.12)*200.,max(0.,load['weight_N']-load['upward_support_N'])*4.,
            max(0.,.65-upper)*(.5 if error<10 else 0.)]
        ring_room=float(np.degrees(guard.qhi[9]-state.qpos[9]))
        finger_loads={name:sum(value['normal_N'] for geom,value in load['by_geom'].items() if (model.body(int(model.geom_bodyid[model.geom(geom).id])).name or '').startswith(name+'_')) for name in contact_fingers}
        if args.ring14_reserve_deg:
            residual=np.r_[residual,max(0.,args.ring14_reserve_deg-ring_room)*2.]
        if args.contact_pressure_target_N:
            residual=np.r_[residual,[max(0.,finger_loads[name]-args.contact_pressure_target_N)*3. for name in contact_fingers]]
        if args.contact_pressure_floor_N:
            residual=np.r_[residual,[max(0.,args.contact_pressure_floor_N-finger_loads[name])*args.contact_pressure_floor_weight for name in contact_fingers]]
        return residual,dict(error_deg=error,tilt_deg=face_up_error_deg(B,5,initial['up']),
            reference_command_error_ticks=float(np.max(abs(motor(state.ctrl)-motor(reference['ctrl'])))),
            planning_phase='orient_and_clear' if transport_complete else 'transport_before_orientation',
            center_target_error_mm=float(np.linalg.norm(C-station)*1000),center_from_source_mm=float(np.linalg.norm(C-start_center)*1000),
            canonical_station_error_mm=float(np.linalg.norm(C-canonical_station)*1000),reference_grip_center_error_mm=float(np.linalg.norm(C-reference_center)*1000),
            upper_screen_mm=upper,minimum_planning_pair_gap_mm=float(min(pair_gaps)),planning_pair_gaps_mm=pair_gaps.tolist(),
            tracking_rad=float(tracking),whole_hand_support=load,RF14_upper_room_deg=ring_room,contact_finger_normals_N=finger_loads)

    def predict(velocity):
        nonlocal planning_calls
        planning_calls+=1
        branch=restore(model,dict({key:getattr(data,key).copy() for key in BOUND},time=data.time));check=Guard(model)
        check.gap=guard.gap;check.longest_gap=guard.longest_gap
        start=motor(branch.ctrl);commands=[];snapshots=[];minimum=float('inf');previous=start.copy()
        for step in range(args.horizon):
            command=np.clip(np.rint(start+(step+1)*velocity).astype(int),command_lo,command_hi)
            if np.max(abs(command-previous))>2:raise AssertionError('Prediction rate changed')
            branch.ctrl[:]=ticks_to_sim(command);previous=command;commands.append(command.copy())
            for _ in range(75):
                mujoco.mj_step(model,branch);stop=check.check(branch)
                if np.linalg.norm(branch.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if stop:rejections.update(stop);return None
            snapshots.append(branch.qpos.copy())
            if args.retain_loaded_contacts and min(contact_forces(branch).values())<=.1:
                rejections.update(['required_contact_at_control_boundary']);return None
            if step+1==args.commit:
                commit_state={key:getattr(branch,key).copy() for key in BOUND}
        residual,ev=evidence(branch)
        # A long prediction still favors progress toward the same task, while
        # preserving a margin before the model's unchanged collision guard.
        score=float(np.linalg.norm(residual))
        return dict(velocity=np.asarray(velocity).copy(),score=score,residual=residual,evidence=ev,
            commands=commands,commit_qpos=snapshots[args.commit-1],commit_state=commit_state)

    start=evidence(data)[1];previous_velocity=np.zeros(16);stop=None;stale=0;best=float('inf');success_window=None;hold_attempts=[];upper_precheck_failures=0
    random=np.random.default_rng(712+args.source_index+int(global_yaw))
    for iteration in range(int(np.ceil(args.steps/args.commit))):
        if len(rows)>=args.steps:break
        if args.hybrid_centering and transport_complete and np.linalg.norm(data.geom_xpos[cube]-station)>.003:
            transport_complete=False;transport_rotation=data.geom_xmat[cube].reshape(3,3).copy();best=float('inf');stale=0
            phase_switches.append(dict(after_goal=len(rows),phase='recenter_preserving_achieved_orientation',center_error_mm=float(np.linalg.norm(data.geom_xpos[cube]-station)*1000),planning_rotation=transport_rotation.tolist()))
        candidates=[];zero=predict(np.zeros(16));current=evidence(data)[0]
        base_residual=zero['residual'] if zero else current;J=np.zeros((len(base_residual),16))
        if zero:candidates.append(zero)
        for joint in range(16):
            ends=[]
            for sign in (-1,1):
                velocity=np.zeros(16);velocity[sid[joint]]=2*sign;candidate=predict(velocity);ends.append(candidate)
                if candidate:candidates.append(candidate)
            if all(item is not None for item in ends):J[:,sid[joint]]=(ends[1]['residual']-ends[0]['residual'])/4
            elif ends[0]:J[:,sid[joint]]=(base_residual-ends[0]['residual'])/2
            elif ends[1]:J[:,sid[joint]]=(ends[1]['residual']-base_residual)/2
        fit=lsq_linear(np.vstack([J,.04*np.eye(16)]),np.r_[-.6*base_residual,np.zeros(16)],bounds=(-2,2),max_iter=100)
        proposal_velocities=[fit.x,fit.x*.5,previous_velocity,(motor(reference['ctrl'])-motor(data.ctrl))/max(args.horizon,1)]
        # Coordinated proposals are tested at every iteration, before a trap,
        # rather than only after all one-joint proposals have already failed.
        for _ in range(16):proposal_velocities.append(np.clip(fit.x+random.normal(0,.85,16),-2,2))
        for velocity in proposal_velocities:
            candidate=predict(np.clip(velocity,-2,2))
            if candidate:candidates.append(candidate)
        if not candidates:stop='no_valid_horizon_proposal';break
        choice=min(candidates,key=lambda item:item['score']);previous_velocity=choice['velocity'];before_score=float(np.linalg.norm(current))
        remaining=min(args.commit,args.steps-len(rows))
        for command in choice['commands'][:remaining]:
            previous=motor(data.ctrl)
            if np.max(abs(command-previous))>2:raise AssertionError('Actual rate changed')
            data.ctrl[:]=ticks_to_sim(command);rows.append(command.copy())
            for _ in range(75):
                mujoco.mj_step(model,data);reasons=guard.check(data)
                if reasons:stop='actual_native_guard:'+','.join(reasons);break
                if np.linalg.norm(data.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop='actual_cube_translation_over80mm';break
            trajectory.append(data.qpos.copy())
            if args.retain_loaded_contacts and min(contact_forces(data).values())<=.1:stop=stop or 'actual_required_contact_at_control_boundary'
            if stop:break
        alignment=float(np.max(abs(data.qpos-choice['commit_qpos']))) if remaining==args.commit and not stop else None
        if alignment is not None and alignment>1e-8:stop='prediction_state_alignment'
        residual,ev=evidence(data);score=float(np.linalg.norm(residual));ev.update(step=len(rows),score=score,predicted_horizon_score=choice['score'],
            prediction_commit_qpos_error=alignment,valid_proposals=len(candidates),chosen_velocity=choice['velocity'].tolist())
        trace.append(ev)
        if score<best-.002:best=score;stale=0
        else:stale+=1
        save_json(folder/'progress.json',dict(mode=args.mode,global_yaw=global_yaw,step=len(rows),actual=ev,planning_calls=planning_calls,elapsed_seconds=time.perf_counter()-began))
        np.savez_compressed(folder/'checkpoint_state.npz',**{key:getattr(data,key) for key in BOUND},time=data.time,
            up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        np.savez_compressed(folder/'checkpoint_goals.npz',start=motor(z['ctrl']),goals=rows)
        print(json.dumps(dict(step=len(rows),error_deg=ev['error_deg'],center_mm=ev['center_target_error_mm'],clearance_mm=ev['minimum_planning_pair_gap_mm'],valid=len(candidates))),flush=True)
        # Recheck the full 4050-state final hold only after actual task entry.
        entry_command_ready=args.entry_command_tolerance_ticks is None or ev['reference_command_error_ticks']<=args.entry_command_tolerance_ticks
        if not stop and entry_command_ready and ev['error_deg']<orientation_tolerance and ev['tilt_deg']<5 and ev['center_target_error_mm']<2:
            certificate=SweepCertificate(model,guard.hand,.3)
            pre_value,pre_bad=certificate.query(data,data.geom_xpos[cube],data.geom_xmat[cube].reshape(3,3))
            if pre_bad or pre_value<=.3:
                upper_precheck_failures+=1
                if stale>=14:stop='strict_upper_acquisition_stalled';break
                continue
            count=supported=certified=0;minimum=float('inf');hold_B=data.geom_xmat[cube].reshape(3,3).copy();hold_C=data.geom_xpos[cube].copy();drift=shift=0.;hold_start=len(rows);loaded_hold={f:0 for f in contact_fingers}
            for cycle in range(54):
                rows.append(motor(data.ctrl))
                for _ in range(75):
                    mujoco.mj_step(model,data);reasons=guard.check(data);count+=1
                    if np.linalg.norm(data.qpos[-7:-4]-initial['initial_cube_center'])>.08:reasons=list(reasons)+['cube_translation_over80mm']
                    C=data.geom_xpos[cube];B=data.geom_xmat[cube].reshape(3,3);lower,bad=certificate.query(data,C,B)
                    minimum=min(minimum,lower);certified+=not bad and lower>.3;load=sample_model(model,data,guard.hand,cube);supported+=load['supported_sample']
                    for f in contact_fingers:
                        force=sum(v['normal_N'] for name,v in load['by_geom'].items() if (model.body(int(model.geom_bodyid[model.geom(name).id])).name or '').startswith(f+'_'));loaded_hold[f]+=force>.1
                    drift=max(drift,orientation_error_deg(B,hold_B));shift=max(shift,float(np.linalg.norm(C-hold_C)*1000))
                    if reasons:stop='final_hold_native_guard:'+','.join(reasons);break
                    if bad:stop='final_hold_upper_certificate';break
                trajectory.append(data.qpos.copy())
                if stop:break
            final=evidence(data)[1]
            success_window=dict(samples=count,certified_samples=certified,min_lower_mm=minimum,support_fraction=supported/max(count,1),angle_drift_deg=drift,center_drift_mm=shift,contact_finger_loaded_fractions={f:n/max(count,1) for f,n in loaded_hold.items()},loaded_contact_hold_required=args.require_loaded_hold,
                after_goal=hold_start,final_error_deg=final['error_deg'],final_center_target_error_mm=final['center_target_error_mm'],
                passed=bool(not stop and count==4050 and certified==4050 and supported/4050>.8 and drift<3 and shift<2 and final['error_deg']<orientation_tolerance and final['tilt_deg']<5 and final['center_target_error_mm']<2 and (not args.require_loaded_hold or all(n/4050>.8 for n in loaded_hold.values()))))
            hold_attempts.append(dict(success_window))
            if success_window['passed']:stop='conditional_task_hold_passed'
            elif stop or len(hold_attempts)>=2:stop=stop or 'final_hold_task_missed'
            else:
                # A settled but task-inaccurate hold remains an actual model
                # state. Resume feedback from it, retaining the failed record.
                stale=0;best=float('inf');continue
            break
        if stop:break
        if not transport_complete and ev['center_target_error_mm']<(1.5 if args.hybrid_centering else 5):
            transport_complete=True;stale=0;best=float('inf')
            phase_switches.append(dict(after_goal=len(rows),phase='orient_and_clear',center_error_mm=ev['center_target_error_mm']))
        if stale>=14:stop='horizon_objective_stalled';break
    np.savez_compressed(folder/'research_goals.npz',start=motor(z['ctrl']),goals=rows)
    np.savez_compressed(folder/'trajectory.npz',qpos=trajectory,control_seconds=.075)
    np.savez_compressed(folder/'end_state.npz',**{key:getattr(data,key) for key in BOUND},time=data.time,
        up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
    result=dict(status='receding_horizon_clear_work_grip_bridge',mode=args.mode,source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),source_index=args.source_index if args.mode=='entry' else None,
        goal_reference=reference_path.relative_to(OUT).as_posix(),goal_reference_sha256=sha(reference_path),global_yaw_target_deg=global_yaw,
        orientation_tolerance_deg=orientation_tolerance,prediction_horizon_cycles=args.horizon,commit_cycles=args.commit,
        transport_first_requested=args.transport_first,transport_phase_completed=transport_complete,explicit_source_state_override=args.start_state,
        center_target_scope=args.station_mode,center_target_world_m=station.tolist(),canonical_station_world_m=canonical_station.tolist(),
        alternative_goal_reference_explicit=args.goal_reference is not None,original_canonical_target_not_reclassified=args.goal_reference is not None,
        center_objective_weight=args.center_weight,joint_reference_objective_weight=args.joint_weight if args.joint_weight is not None else (.025 if args.mode=='next' else .06),
        ring14_reserve_objective_deg=args.ring14_reserve_deg,contact_pressure_objective_target_N=args.contact_pressure_target_N,
        contact_pressure_floor_objective_N=args.contact_pressure_floor_N,ring_thumb_planning_pairs_included=args.include_ring_thumb_pairs,
        contact_fingers=contact_fingers,loaded_contact_hold_required=args.require_loaded_hold,middle_thumb_planning_pair_included=args.include_middle_thumb_pair,
        contact_pressure_floor_weight=args.contact_pressure_floor_weight,retain_loaded_contacts_at_control_boundaries=args.retain_loaded_contacts,
        relief_terms_are_planning_costs_not_native_guard_changes=True,
        ID6_upper_command_reserve_ticks=args.ID6_upper_reserve_ticks,command_reserve_does_not_change_native_joint_or_guard_limits=True,
        entry_command_tolerance_ticks=args.entry_command_tolerance_ticks,entry_hand_alignment_is_additional_task_requirement_not_guard_change=True,
        hybrid_centering_requested=args.hybrid_centering,phase_switches=phase_switches,final_global_orientation_target_not_changed_by_waypoints=True,
        proactive_clearance_margin_mm=args.margin_mm,planning_pairs=[list(pair) for pair in pair_names],source_model_provenance=provenance,
        planning_pair_cost='bounded_axes_not_native_collision_or_final_certificate',planning_pair_cost_source_sha256=sha(OUT/'planning_pair_gap.py'),
        sliding_friction_hypothesis=args.sliding_friction,only_sliding_friction_entry_changed=args.sliding_friction is not None,middle_self_planning_pair_included=args.include_middle_self_pair,
        steps=len(rows),stop=stop,start=start,final=evidence(data)[1],trace=trace,planning_calls=planning_calls,prediction_rejections=dict(rejections),final_hold=success_window,
        hold_attempts=hold_attempts,strict_upper_acquisition_failures=upper_precheck_failures,
        conditional_bridge_and_hold_passed=bool(success_window and success_window['passed']),best_sampled_error_deg=min([start['error_deg']]+[row['error_deg'] for row in trace]),
        actual_state_resets_after_start=0,original_native_guards_and_joint_limits_preserved=True,worst=guard.worst,
        model_based_teacher_not_SAC=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False,
        full_initial_placement_two_turn_cycle_passed=False,elapsed_seconds=time.perf_counter()-began)
    save_json(folder/'review.json',result)
    print(json.dumps(dict(mode=args.mode,steps=len(rows),stop=stop,best_error_deg=result['best_sampled_error_deg'],passed=result['conditional_bridge_and_hold_passed'])),flush=True)


if __name__=='__main__':main()
