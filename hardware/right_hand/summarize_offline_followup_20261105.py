"""Bind the offline follow-up results, preserving failures and task distinctions."""
import hashlib,json,math
from pathlib import Path
O=Path(__file__).resolve().parent/'right_hand_rotation_20261105/canonical_body_grip';R=O.parents[1]
sources={}
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):
    sources[p.relative_to(R).as_posix()]=sha(p);d=json.loads(p.read_bytes())
    if 'devices_opened' in d:assert d['devices_opened'] is False
    if 'hardware_execution_allowed' in d:assert d['hardware_execution_allowed'] is False
    return d
def group(pattern):return [(p,read(p)) for p in sorted(O.glob(pattern))]
def clean(x):
    if isinstance(x,float) and not math.isfinite(x):return None
    if isinstance(x,dict):return {k:clean(v) for k,v in x.items()}
    if isinstance(x,list):return [clean(v) for v in x]
    return x

def main():
    faces=read(O/'face_hold_hypotheses/review.json');symmetry=read(O/'cube_symmetry_review.json');regression=read(O/'green_default_regression.json');photo=read(O/'model_discrepancy/photo_angle_sensitivity.json');layer=read(O/'articulated_upper_layer/validation.json')
    assert len(faces['cases'])==6 and len(symmetry['cases'])==24 and regression['passed']
    for case in layer['cases']:
        p=R/case['source'];assert sha(p)==case['sha256'];read(p)
    audits=[]
    for p,d in group('articulated_upper_layer/*_friction_audit/review.json'):
        audits.append(dict(source=p.relative_to(R).as_posix(),bearing_parameter_nm=d['bearing_frictionloss_hypothesis_nm'],maximum_external_torque_nm=d['maximum_external_layer_torque_nm'],maximum_hinge_constraint_torque_nm=d['maximum_absolute_hinge_constraint_torque_nm'],frictionloss_disabled=d['frictionloss_disabled'],passed=d['load_filter_passed']))
    search={}
    support=[c for p,d in group('support_release_search/*/review.json') for c in d['cases']]
    search['index_support_release']=dict(cases=len(support),safety_passed=sum(c['dynamic']['safety_passed'] for c in support),target90_passed=sum(c['target90_filter_passed'] for c in support))
    timing=[c for p,d in group('grasp_timing_search/*/review.json') for c in d['cases']]
    search['initial_grasp_timing']=dict(cases=len(timing),safety_passed=sum(c['dynamic']['safety_passed'] for c in timing),target90_and_common_passed=sum(c['whole90_and_common_grip_filter_passed'] for c in timing))
    rolling=group('rolling_feedback/*/review.json')
    search['simulated_feedback']=dict(cases=len(rolling),completed90_and_common_passed=sum(d['common90_filter_passed'] for p,d in rolling),results=[dict(source=p.relative_to(R).as_posix(),safety=d['safety_passed'],yaw_deg=d['final_yaw_deg'],guard_stop=d['guard_stop'],samples=d['samples']) for p,d in rolling],perfect_object_feedback_not_a_live_controller=True)
    static=group('sliding_workspace_grip/*/review.json')
    search['workspace_static_proposals']=dict(cases=len(static),accepted_static_only=sum(d['proposal_accepted'] for p,d in static),accepted=[p.relative_to(R).as_posix() for p,d in static if d['proposal_accepted']],not_free_motion_evidence=True)
    transitions=[(p,c) for p,d in group('workspace_transition/*/review.json') for c in d['cases']]
    search['workspace_free_transitions']=dict(cases=len(transitions),safety_passed=sum(c['dynamic']['safety_passed'] for p,c in transitions),common_grip_passed=sum(c['grip_task_passed'] for p,c in transitions))
    lifting=group('workspace_lift_yaw/*/review.json')
    search['workspace_lifting']=dict(cases=len(lifting),target90_and_hold_passed=sum(d['goal90_and_hold_reached'] for p,d in lifting),results=[dict(source=p.relative_to(R).as_posix(),yaw_deg=d['final_yaw_deg'],lift_mm=d['final_lift_mm'],guard_stop=d['guard_stop'],search_stop=d['search_stop']) for p,d in lifting])
    wide=group('wide_initial_yaw/*/review.json')
    search['wide_initial_yaw']=dict(cases=len(wide),target90_and_common_passed=sum(d['quarter_turn90_and_common_filter_passed'] for p,d in wide),results=[dict(source=p.relative_to(R).as_posix(),initial_yaw_deg=d['initial_yaw_hypothesis_deg'],final_turn_deg=d['dynamic']['final_yaw_from_placement_deg'] if d['dynamic'] else None,reasons=d['common_grip_task_reasons']) for p,d in wide])
    index=[c for p,d in group('index_assisted_yaw/*/review.json') for c in d['cases']]
    search['index_assisted_yaw']=dict(cases=len(index),safety_passed=sum(c['dynamic']['safety_passed'] for c in index),target90_direction_passed=sum(c['target90_direction_filter_passed'] for c in index),best_yaw_deg=max(c['whole_yaw_deg'] for c in index),cases_include_baseline_and_seeded_previous_proposals=True)
    force_scope=read(O/'motor_model_force_scope.json');caps=group('torque_cap_sensitivity/*/review.json')
    cap_summary=dict(cases=len(caps),task_passed=sum(d['task_passed'] for p,d in caps),nominal_gain_and_friction_only=True,real450mA_to_torque_not_calibrated=True,results=[dict(source=p.relative_to(R).as_posix(),hypothetical_cap_Nm=d['hypothetical_cap_each_joint_Nm'],task=d['task_passed'],observed_max_torque_Nm=d['observed_maximum_actuator_force_Nm'],reasons=d['reasons']) for p,d in caps])
    cap_loads=[c for p,d in group('capped_grip_load/*/review.json') for c in d['cases']]
    cap_load_summary=dict(cases=len(cap_loads),passed=sum(c['load_filter_passed'] for c in cap_loads),nonzero_load_cases=sum(c['hypothetical_reaction_torque_nm']!=0 for c in cap_loads),nonzero_load_passed=sum(c['load_filter_passed'] for c in cap_loads if c['hypothetical_reaction_torque_nm']!=0),rigid_whole_cube_hypothetical_load_only=True)
    static=read(O/'contact_wrench_audit/cap0.15_native_cone/review.json')
    static_summary=dict(cases=len(static['cases']),feasible=sum(c['static_force_allocation_feasible'] for c in static['cases']),hypothetical_cap_Nm=.15,uses_native_pyramidal_friction=static['force_allocation_uses_same_native_pyramidal_cone'],actuator_balance_reconstruction_error_ticks=static['actuator_balance_reconstruction_error_ticks'],not_a_dynamic_path_or_control_authority_proof=True)
    directions=read(O/'contact_wrench_audit/control_directions.json')
    controlled=group('quasistatic_wrench_path/*/review.json');control_results=[]
    for p,d in controlled:
        v=d.get('recorded_goal_replay_verification');constant=bool(d.get('last134_goals_proved_constant',False));passed=bool(v and v['safety_passed'] and v['whole_grip_task_passed'] and constant)
        control_results.append(dict(source=p.relative_to(R).as_posix(),task=d['task'],hybrid_motion=d.get('hybrid_geometric_motion_and_force_preload',False),revision=d.get('revision','legacy'),constant134=constant,replayed_common_grip_passed=passed,checkpoint_quarter_passed=bool(d.get('checkpoint_quarter_filter_passed',False)),final_yaw_deg=d['final_yaw_deg'],final_center_shift_mm=d['final_center_shift_mm'],guard_stop=d['guard_stop'],search_stop=d['search_stop'],intermediate_target_yaw_deg=d.get('intermediate_target_yaw_deg',90),planned_hand_Y_translation_mm=d.get('planned_hand_Y_translation_mm',0),full_initial_placement_not_replayed=True))
    search['quasistatic_wrench_control']=dict(cases=len(controlled),fixed_hold_replay_passed=sum(c['replayed_common_grip_passed'] for c in control_results if c['task']=='hold'),checkpoint_quarter_passed=sum(c['checkpoint_quarter_passed'] for c in control_results if c['task']=='quarter'),results=control_results,perfect_simulated_feedback_not_live_policy=True,software_debug_exceptions_not_counted_as_motion_trials=True)
    near=read(O/'near_quarter_full_replay/review.json');v=near['dynamic']
    for field,hashfield in (('source_model','source_model_sha256'),('source_initial_state','source_initial_state_sha256'),('source_controller_review','source_controller_review_sha256')):
        p=R/near[field];assert sha(p)==near[hashfield];sources[p.relative_to(R).as_posix()]=sha(p)
    for file in ('candidate.npz','trajectory.npz','end_state.npz'):
        p=O/'near_quarter_full_replay'/file;sources[p.relative_to(R).as_posix()]=sha(p)
    near_summary=dict(source='near_quarter_full_replay/review.json',total_goals=near['total_goals'],safety_passed=v['safety_passed'],completed=v['completed_goals'],whole_cube_yaw_deg=v['final_yaw_from_placement_deg'],selected_top_tilt_deg=v['final_up_tilt_deg'],fixed_hold_seconds=v['full_fixed_hold']['seconds'],fixed_hold_yaw_drift_deg=v['full_fixed_hold']['yaw_drift_deg'],fixed_hold_shift_mm=v['full_fixed_hold']['center_shift_mm'],common_grip_passed=near['quarter_and_common_grip_passed'],reasons=near['quarter_and_common_grip_reasons'],initial_placement_through_fixed_hold_replayed=True,free_cube_not_reset=True,physical_rotation_not_verified=True)
    postturn=[]
    for p,d in group('postturn_regrasp*/review.json'):
        for c in d['cases']:postturn.append(dict(source=p.relative_to(R).as_posix(),finger=c['finger'],mode=c['mode'],static_proposal_accepted=c['static_proposal_accepted'],dynamic_grip_passed=any(v.get('result',{}).get('whole_grip_task_passed',False) for v in c['dynamic'])))
    search['postturn_regrasp']=dict(cases=len(postturn),static_accepted=sum(c['static_proposal_accepted'] for c in postturn),dynamic_grip_passed=sum(c['dynamic_grip_passed'] for c in postturn),results=postturn)
    retreat=read(O/'postturn_middle_clearance/review.json');workspace=read(O/'postturn_middle_clearance/workspace_diagnostic.json')
    thumb=[c for p,d in group('postturn_thumb_clearance/review.json')+group('postturn_thumb_coupled/review.json') for c in d['cases']]
    search['postturn_thumb_geometry']=dict(cases=len(thumb),original_dynamic_safety_passed=sum(c['dynamic']['safety_passed'] for c in thumb),geometry_and_named_pressure_screen_passed=sum(c['palm_index_thumb_geometry_screen_passed'] for c in thumb),screen_is_not_force_closure_or_load_certification=True,upper_space_only_passed=sum(c['dynamic']['settled_tail']['separation_certificate_bad_steps_1ms']==0 and c['dynamic']['settled_tail']['minimum_separating_plane_gap_1ms_mm']>.3 for c in thumb),closest_remaining_upper_obstruction=workspace['closest_geoms'][:3],middle_retreat_yaw_deg=retreat['dynamic']['final_yaw_from_placement_deg'],middle_retreat_safety=retreat['dynamic']['safety_passed'])
    thumb_ik=[c for p,d in group('postturn_thumb_ik/review.json') for c in d['cases']]
    search['postturn_thumb_static_ik']=dict(cases=len(thumb_ik),static_accepted=sum(c['static_accepted'] for c in thumb_ik),free_replays=sum(len(c['dynamic']) for c in thumb_ik),static_geometry_not_free_motion=True)
    palm_loads=[]
    for p,d in group('palm_supported_quarter_load*/review.json'):
        palm_loads.append(dict(source=p.relative_to(R).as_posix(),downward_N=d.get('hypothetical_downward_force_N',0.),cases=len(d['cases']),functional_probe_passed=sum(c['functional_reaction_probe_passed'] for c in d['cases']),results=[dict(torque_Nm=c['dynamic']['hypothetical_reaction_torque_nm'],passed=c['functional_reaction_probe_passed'],yaw_drift_deg=c['dynamic']['max_yaw_change_deg'],shift_mm=c['dynamic']['max_center_shift_mm'],reasons=c['functional_reasons']) for c in d['cases']]))
    search['palm_supported_functional_load']=dict(cases=sum(c['cases'] for c in palm_loads),groups=palm_loads,nominal_model_only=True,old_named_MF_TH_grip_filter_unchanged=True,left_hand_geometry_and_force_capacity_not_verified=True,static_support_does_not_prove_full_load_resistance=True)
    # Keep proposal counts separate from free-cube goal replays. A sampled IK
    # workspace point is not a motion trial, and duplicated paths are not
    # independent successes.
    recent_patterns=('thumb_global_workspace/*/review.json','index_countergrip_workspace/*/review.json','index_lower_regrasp_workspace/*/review.json','countergrip_level/*/review.json','middle_support_transfer*/review.json','middle_tip_support_transfer*/review.json','initial_station*replay/review.json','thumb_corner_support/review.json','lower_countergrip_station*/*review.json','countergrip_fixed_prefix/review.json','*_side_brace/review.json','pinch_transfer_turn/*/review.json')
    recent=[];goal_hashes={}
    for pattern in recent_patterns:
        for p,d in group(pattern):
            cases=d.get('cases',[])
            # Non-nested feedback runs have one complete raw report each.
            proposal_cases=[c for c in cases if any(k in c for k in ('accepted_static_only','static_force_feasible','static_proposal_accepted','static_accepted'))]
            motions=[];rejected=0;baseline_reused=0
            for c in cases:
                dynamic=c.get('dynamic',[])
                if isinstance(dynamic,dict):motions.append(dynamic);baseline_reused+=int(c.get('baseline_reused',False))
                else:
                    for trial in dynamic:
                        rejected+=int(trial.get('native_goal_rejected',False))
                        if 'result' in trial:motions.append(trial['result'])
                rejected+=int(c.get('native_goal_rejected',False))
            entry=dict(source=p.relative_to(R).as_posix(),status=d['status'],static_proposals=len(proposal_cases),static_accepted=sum(bool(c.get('accepted_static_only',c.get('static_force_feasible',c.get('static_proposal_accepted',c.get('static_accepted',False))))) for c in proposal_cases),nested_free_replays=len(motions),nested_original_safety_passed=sum(v['safety_passed'] for v in motions),native_goal_proposals_rejected=rejected,baseline_reports_reused=baseline_reused,checkpoint_complete=bool(d.get('checkpoint90_and_new_grip_passed',False)),guard_stop=d.get('guard_stop'),search_stop=d.get('search_stop'),final_yaw_deg=d.get('final_yaw_deg'),samples=d.get('samples'),phase=d.get('phase'))
            replay=d.get('recorded_goal_replay')
            if replay:
                entry['fixed_replay']=dict(safety_passed=replay['safety_passed'],yaw_deg=replay['final_yaw_from_placement_deg'],tilt_deg=replay['final_up_tilt_deg'],contact_fraction=replay['settled_tail']['opposed_lower_body_contact_fraction'],upper_gap_mm=replay['settled_tail']['minimum_separating_plane_gap_1ms_mm'],reasons=d['reasons'])
            # Bind goals and endpoint states alongside summaries so an accepted
            # pinned pose cannot be mistaken for the replayed cube position.
            for raw in sorted(p.parent.rglob('*.npz')):
                name=raw.relative_to(R).as_posix();sources[name]=sha(raw)
                import numpy as np
                with np.load(raw) as z:
                    if 'goals' in z:
                        key=hashlib.sha256(np.asarray(z['goals'],dtype=np.int64).tobytes()).hexdigest()
                        if name not in goal_hashes.setdefault(key,[]):goal_hashes[key].append(name)
            recent.append(entry)
    search['postturn_contact_replanning_20261106']=dict(cases=len(recent),reports=recent,unique_recorded_goal_arrays=len(goal_hashes),recorded_goal_files=sum(map(len,goal_hashes.values())),identical_goal_array_groups=[v for v in goal_hashes.values() if len(v)>1],static_proposals_are_not_free_motion_trials=True,full_initial_placement_not_replayed_for_checkpoint_continuations=True)
    temporary=read(O/'temporary_handoff_load/review.json')
    search['temporary_handoff_load']=dict(cases=len(temporary['cases']),passed=sum(c['load_filter_passed'] for c in temporary['cases']),source_yaw_deg=temporary['source_yaw_deg'],original90_target_not_passed=True,loads=[dict(torque_Nm=c['hypothetical_reaction_torque_nm'],passed=c['load_filter_passed'],yaw_drift_deg=c['max_yaw_change_deg'],shift_mm=c['max_center_shift_mm'],gap_mm=c['minimum_separating_gap_1ms_mm'],reasons=c['reasons']) for c in temporary['cases']],not_full_articulated_layer_or_real_left_hand=True)
    for p in (O/'temporary_handoff_load').glob('*.npz'):sources[p.relative_to(R).as_posix()]=sha(p)
    protocol=read(O/'temporal_handoff_review.json')
    assert protocol['synthetic_cycle_only'] and not protocol['actual_loaded_motion_connection_implemented'] and all(c['left_access_denied'] for c in protocol['saved_candidate_denials'])
    # New nominal task evidence remains separate from old robustness matrices.
    continuous=read(O/'continuous_handoff/review.json');assert continuous['quarter_and_handoff_passed'] and continuous['all_cube_dynamics_continuous_no_resets']
    cv=continuous['dynamic'];ct=cv['settled_tail'];loads=continuous['hypothetical_load_cases']
    current=dict(source='continuous_handoff/review.json',candidate='continuous_handoff/candidate.npz',candidate_sha256=continuous['candidate_sha256'],nominal_cases=1,quarter_and_handoff_passed=True,target_window_degrees=[85,95],exact90_not_proved=True,total_goals=continuous['total_goals'],nominal_seconds=continuous['nominal_seconds'],yaw_deg=cv['final_yaw_from_placement_deg'],tilt_deg=cv['final_up_tilt_deg'],orientation_error_deg=continuous['orientation_error_deg'],upper_gap_mm=ct['minimum_separating_plane_gap_1ms_mm'],lower_contact_fraction=ct['opposed_lower_body_contact_fraction'],initial_placement_to_handoff_continuous=True,all_goal_seams_exact=continuous['all_goal_seams_exact'],all_1ms_dynamic_guards_passed=cv['safety_passed'],maximum_checkpoint_qpos_error=max(c['maximum_qpos_error'] for c in continuous['source_checkpoint_state_comparisons']),load_cases=len(loads),load_passed=sum(c['load_filter_passed'] for c in loads),load_results=[dict(torque_Nm=c['hypothetical_reaction_torque_nm'],passed=c['load_filter_passed'],yaw_drift_deg=c['max_yaw_change_deg'],shift_mm=c['max_center_shift_mm'],gap_mm=c['minimum_separating_gap_1ms_mm'],reasons=c['reasons']) for c in loads],new_robustness_matrix_not_run=True,not_real_hardware_success=True)
    new_reports=set()
    for pattern in ('supported_handoff*/**/review.json','continuous_handoff/review.json','continuous_retiming/*/review.json','handoff_preload/*/review.json','feedback_body_hold/*/review.json','static_handoff_equilibrium/review.json','three_contact_handoff*/review.json','independent_balanced_grip/review.json'):
        new_reports.update(O.glob(pattern))
    newer=[]
    for p in sorted(new_reports):
        d=read(p);entry=dict(source=p.relative_to(R).as_posix(),status=d['status'],compiled_unique_rollouts=d.get('unique_rollouts'),quarter_and_handoff_passed=d.get('quarter_and_handoff_passed',d.get('checkpoint90_and_grip_passed')),coarse_search_is_not_full_dynamic_verification=True)
        if 'not_a_live_camera_or_SAC_controller' in d:
            entry.update(perfect_model_feedback_only=True,cases=len(d['cases']),passed=sum(c['passed'] for c in d['cases']),failures=[dict(torque_Nm=c['torque_Nm'],reasons=c['reasons']) for c in d['cases'] if not c['passed']])
        newer.append(entry)
        for raw in sorted(p.parent.rglob('*.npz')):sources[raw.relative_to(R).as_posix()]=sha(raw)
    for key in ('source_model','source_initial_state'):
        p=R/continuous[key];assert sha(p)==continuous[key+'_sha256'];sources[p.relative_to(R).as_posix()]=sha(p)
    evidence=read(O/'continuous_handoff/evidence/manifest.json')
    assert evidence['source_review_sha256']==sha(O/'continuous_handoff/review.json') and not evidence['physical_video']
    for name in evidence['images']+[v['file'] for v in evidence['videos']]:
        p=O/'continuous_handoff/evidence'/name;sources[p.relative_to(R).as_posix()]=sha(p)
    search['supported_handoff_and_feedback_20261106']=dict(cases=len(newer),reports=newer,counts_are_report_counts_not_independent_success_rates=True)
    sensitivity=read(O/'continuous_sensitivity/review.json')
    for p,h in sensitivity['source_documents_sha256'].items():assert sha(R/p)==h;sources[p]=h
    current.update(new_robustness_matrix_not_run=False,full_combined_robustness_matrix_not_run=True,one_factor_sensitivity=sensitivity,turn_axis='world_up',top_face_id=5,other_axis_top_face_changes_not_verified=True)
    progress_media=read(O/'countergrip_progress/manifest.json')
    for item in progress_media['evidence']:
        assert sha(R/item['source_review'])==item['source_review_sha256']
        for image in item['images']:
            p=O/'countergrip_progress'/image;sources[p.relative_to(R).as_posix()]=sha(p)
        if 'video' in item:
            p=O/'countergrip_progress'/item['video']['file'];assert sha(p)==item['video']['sha256'];sources[p.relative_to(R).as_posix()]=sha(p)
    assert search['simulated_feedback']['completed90_and_common_passed']==0
    assert search['initial_grasp_timing']['target90_and_common_passed']==0
    report=dict(status='offline_followup_task_separated_and_source_bound',face_hold_hypotheses=dict(cases=6,passed=sum(c['task_passed'] for c in faces['cases']),seconds_each=10.05,independent_initial_states_not_reorientation_paths=True),cube_label_symmetry=dict(cases=24,passed=sum(c['passed'] for c in symmetry['cases']),static_only_not24_dynamic_trials=True),unchanged_default_green_regression=dict(passed=regression['passed'],matched_scalar_fields=regression['compared_scalar_fields']),photo_angle_sensitivity=dict(cases=len(photo['cases']),summary=photo['summary'],not_a_statistical_confidence_interval=True),research=search,articulated_upper_layer=dict(cases=layer['total_cases'],passed=layer['passed'],maximum_lower_body_yaw_deg=max(c['body_yaw_deg'] for c in layer['cases']),maximum_lower_body_shift_mm=max(c['body_shift_mm'] for c in layer['cases']),minimum_upper_gap_mm=min(c['gap_mm'] for c in layer['cases']),maximum_external_layer_torque_nm=max(c['maximum_external_layer_torque_nm'] for c in layer['cases']),actual_left_hand_geometry_absent=True,bearing_and_load_are_uncalibrated_hypotheses=True,full_placement_not_replayed_with_separate_layer=True,not_whole_cube_reorientation90=True),bearing_friction_audits=audits,preferred_offline_common_grip_unchanged=True,new_valid_whole_cube90_path=False,new_physical_execution_instructions_created=False,hardware_execution_allowed=False,devices_opened=False,source_documents_sha256=sources)
    report.update(near_quarter_full_initial_replay=near_summary,torque_capped_full_path=cap_summary,torque_capped_load_hypotheses=cap_load_summary,native_static_contact_force_audit=static_summary,motor_model_force_scope=force_scope,contact_translation_control_directions=dict(contact_points=len(directions['contacts']),index_base_rank=min(c['actuated_point_translation_rank'] for c in directions['contacts'] if c['other'].startswith('if_')),not_a_full_robot_controllability_analysis=True),current_nominal_task_candidate=current,new_valid_whole_cube90_path=continuous['quarter_and_handoff_passed'],new_path_scope='one nominal model in90plusminus5 window; high-load, robustness, next rotation and real verification incomplete',preferred_offline_common_grip_unchanged=False)
    (O/'offline_followup.json').write_text(json.dumps(clean(report),indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(dict(faces=report['face_hold_hypotheses'],layer=report['articulated_upper_layer'],research_groups={k:v['cases'] for k,v in search.items()},source_reports=len(sources),nominal_quarter_and_handoff=current['quarter_and_handoff_passed'],physical_completion=False)),flush=True)
if __name__=='__main__':main()
