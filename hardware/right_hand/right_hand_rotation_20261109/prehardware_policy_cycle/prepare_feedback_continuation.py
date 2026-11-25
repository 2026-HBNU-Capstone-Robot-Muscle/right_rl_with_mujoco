"""Prepare pending offline calculations and audit stored segment durations.

This host-side preparation does not execute the WSL calls rejected by approval
review and does not infer physical success from saved model evidence.
"""
from pathlib import Path
import json
import hashlib
import numpy as np

OUT=Path(__file__).resolve().parent;BASE=OUT/'next_cycle/handoff_refinement_20261110';FOLDER=BASE/'reposition_then_second_rotation_20261111'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    rows=[]
    completed_reports={}
    for name in ('robust_mu025_declared_face_feedback_v4','robust_mu025_declared_face_feedback_v5','robust_mu025_feedback_gait_push_v11','robust_mu025_feedback_pressure_v12','robust_mu025_feedback_bounded_fit_v13','robust_mu025_feedback_net_moment_v14','robust_mu025_late_MF_feedback_v26','robust_mu025_late_TH_feedback_v34','robust_mu025_late_TH_normal_release_v35'):
        path=BASE/name/'review.json'
        if not path.exists():continue
        r=json.loads(path.read_text(encoding='utf-8'));completed_reports[path.relative_to(OUT).as_posix()]=sha(path)
        for case in r['rows']:
            rows.append(dict(study=name,path=case['path'],steps=case['steps'],final180_error_deg=case['final180_error_deg'],stop=case['stop'],hold_samples=case['hold_samples'],actual_contact=case['actual_contact'],worst=case['worst']))
    other_studies=[]
    for name in ('robust_mu025_late_RF_geometry_v15','robust_mu025_late_TH_geometry_v15','robust_mu025_late_TH_standoff_v16','robust_mu025_TH_relief_v18','robust_mu025_late_whole_hand_v19','robust_mu025_late_contact_floor_v20','late_teacher_checkpoint_for_contact_floor','placement_one_tick_reserve_20261111','robust_mu025_late_MF_reset_v21','robust_mu025_late_MF_geometry_v22','robust_mu025_late_opposite_face_plan_v23','robust_mu025_late_same_face_plan_v23','robust_mu025_late_MF_free_route_v24','robust_mu025_late_MF_supported_route_v25','robust_mu025_late_TH_geometry_v28','robust_mu025_late_TH_free_plan_v29','robust_mu025_late_TH_free_route_v30','robust_mu025_late_TH_lag_route_v31','robust_mu025_late_TH_normal_retraction_v32','robust_mu025_late_TH_retracted_route_v33'):
        path=BASE/name/'review.json'
        if path.exists():
            r=json.loads(path.read_text(encoding='utf-8'));completed_reports[path.relative_to(OUT).as_posix()]=sha(path);other_studies.append(dict(study=name,status=r['status'],steps=r.get('steps'),stop=r.get('stop'),final180_error_deg=r.get('final180_error_deg',r.get('final',{}).get('error_deg')),passed=r.get('rotation_and_clear_hold_at_reached_location_passed',r.get('conditional_bridge_and_hold_passed',False))))
            if r['status']=='receding_horizon_clear_work_grip_bridge' and r['stop']=='horizon_objective_stalled':
                rows.append(dict(study=name,path=path.parent.relative_to(OUT).as_posix(),steps=r['steps'],final180_error_deg=r['final']['error_deg'],stop=r['stop'],hold_samples=0,actual_contact=None,worst=r['worst'],stable_partial_model_case_despite_objective_stall=True))
    first=BASE/'robust_cycle_preparation_20261111/low_friction_first_hold_state_v4/end_state.npz'
    parts=[first,BASE/'robust_mu025_reposition_to_nominal_entry_v1/end_state.npz',BASE/'reposition_then_second_rotation_20261111/finger_alignment_orders_v1/RF_IF_MF_TH/end_state.npz',BASE/'robust_mu025_entry_orientation_repair_v3/end_state.npz',BASE/'robust_mu025_reposition_MF_contact_arrival_v1/axis0_sign+1_force0/end_state.npz',BASE/'robust_mu025_reposition_MF_reset_gait_v1/pip-30_dip15_mcp10/end_state.npz',BASE/'robust_mu025_free_MF_standoff_route_v1/end_state.npz',BASE/'robust_mu025_declared_face_feedback_v4/force0.5/end_state.npz',BASE/'robust_mu025_feedback_reset_v7/pip-15_dip0_mcp0/end_state.npz',BASE/'robust_mu025_feedback_free_route_v10/end_state.npz',BASE/'robust_mu025_feedback_gait_push_v11/force0.8/end_state.npz']
    provenance=json.loads((BASE/'robust_mu025_feedback_free_route_v10/review.json').read_text(encoding='utf-8'))['model_provenance'];initial=OUT.parents[1]/provenance['initial_path'];assert sha(initial)==provenance['initial_sha256'];previous_time=float(np.load(initial,allow_pickle=False)['time']);timing=[]
    for i,ep in enumerate(parts):
        z=np.load(ep,allow_pickle=False);time=float(z['time']);samples=int(round((time-previous_time)/.001));previous_time=time
        count=2478 if i==0 else len(np.load(ep.parent/'research_goals.npz',allow_pickle=False)['goals'])
        last=samples-(count-1)*75;assert 1<=last<=75,(ep,last)
        timing.append(dict(path=ep.relative_to(OUT).as_posix(),sha256=sha(ep),goals=count,samples=samples,final_command_duration_ms=last))
    scripts=['servo_declared_face_approach.py','replay_low_friction_feedback_chain.py']
    jobs=[dict(id='pressure_v12',script=scripts[0],arguments=['--source-state','next_cycle/handoff_refinement_20261110/robust_mu025_feedback_gait_push_v11/force0.8/end_state.npz','--geometry','next_cycle/handoff_refinement_20261110/robust_mu025_feedback_gait_geometry_v8/review.json','--profile','axis2_sign-1','--tag','robust_mu025_feedback_pressure_v12','--pressures','1.5','--steps','1000','--joint-reserve-ticks','4'],expected_report='robust_mu025_feedback_pressure_v12/review.json'),dict(id='exact_chain',script=scripts[1],arguments=['--tag','robust_mu025_feedback_chain_exact_replay_v1'],expected_report='robust_mu025_feedback_chain_exact_replay_v1/review.json'),dict(id='complete_75ms_chain',script=scripts[1],arguments=['--tag','robust_mu025_feedback_chain75ms_replay_v1','--complete-75ms'],expected_report='robust_mu025_feedback_chain75ms_replay_v1/review.json')]
    for job in jobs:
        result=BASE/job['expected_report'];snapshot=result.parent/('source_'+job['script']);job.update(script_sha256=sha(OUT/job['script']),completed=result.exists(),execution_requires_working_approval_review=True)
        if result.exists():
            job['result_sha256']=sha(result);completed_reports[result.relative_to(OUT).as_posix()]=sha(result)
            if snapshot.exists():job.update(executed_source_snapshot=snapshot.relative_to(OUT).as_posix(),executed_source_sha256=sha(snapshot))
    for name in ('robust_mu025_final_clear_hold_v37','robust_mu025_final_pose_space_v38'):
        path=BASE/name/'review.json'
        if path.exists():
            r=json.loads(path.read_text(encoding='utf-8'));completed_reports[path.relative_to(OUT).as_posix()]=sha(path)
            other_studies.append(dict(study=name,status=r['status'],steps=r.get('steps'),stop=r.get('stop'),passed=r.get('conditional_bridge_and_hold_passed',False)))
            if r.get('conditional_bridge_and_hold_passed'):
                rows.append(dict(study=name,path=path.parent.relative_to(OUT).as_posix(),steps=r['steps'],final180_error_deg=r['final_hold']['final_error_deg'],stop=r['stop'],hold_samples=r['final_hold']['samples'],actual_contact=None,worst=r['worst'],conditional_final_clear_hold_passed=True))
    replays=[]
    for name in ('robust_mu025_feedback_chain_exact_replay_v1','robust_mu025_feedback_chain75ms_replay_v1','robust_mu025_late_whole_hand75ms_replay_v27','robust_mu025_complete_cycle75ms_replay_v39'):
        path=BASE/name/'review.json'
        if path.exists():
            r=json.loads(path.read_text(encoding='utf-8'));completed_reports[path.relative_to(OUT).as_posix()]=sha(path);replays.append(dict(path=path.relative_to(OUT).as_posix(),steps=r['steps'],stop=r['stop'],all1ms_samples=r['all1ms_samples'],original75ms_schedule_used=r['original75ms_schedule_used'],final180_error_deg=r['final180_error_deg'],actual_state_resets_after_initial_placement=r['actual_state_resets_after_initial_placement'],full_two_turn_two_hold_success=r['full_two_turn_two_hold_success']))
    selected=min((r for r in rows if not r['stop'] or r.get('stable_partial_model_case_despite_objective_stall') or r.get('conditional_final_clear_hold_passed')),key=lambda r:(not r.get('conditional_final_clear_hold_passed',False),r['final180_error_deg']))
    report=dict(status='offline_continuation_resumed',completed_model_studies=rows,other_studies=other_studies,selected_stable_partial_model_case=selected,completed_reports_sha256=completed_reports,segment_timing=timing,all_segment_durations_75ms=all(r['final_command_duration_ms']==75 for r in timing),timing_is_saved_timestamps_not_dynamic_replay=True,independent_replays=replays,pending_jobs=jobs,approval_usage_block_current=False,prior_approval_review_failure='The earlier usage limit prevented execution in the previous turn; authorized WSL calls now execute successfully.',remaining_work=['Finish the low-friction second turn and certify the final supported clear hold','Replay newer continuation suffixes from the complete75ms original-placement state','Recheck the placement perturbation and preserve finite robustness scope','Validate physical observation registration before any physical run'],new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False)
    (FOLDER/'continuation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(status=report['status'],latest_error_deg=rows[-1]['final180_error_deg'],steps=sum(r['goals'] for r in timing),timing_exceptions=[r for r in timing if r['final_command_duration_ms']!=75],pending_jobs=[j['id'] for j in jobs if not j['completed']]),ensure_ascii=False))

if __name__=='__main__':main()
