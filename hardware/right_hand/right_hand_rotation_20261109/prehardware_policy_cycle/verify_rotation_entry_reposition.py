"""Verify the actual entry mismatch and no-reset reposition experiments."""
from pathlib import Path
import hashlib
import json
import numpy as np

OUT=Path(__file__).resolve().parent;BASE=OUT/'next_cycle/handoff_refinement_20261110';FOLDER=BASE/'reposition_then_second_rotation_20261111'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_text(encoding='utf-8'))


def verify():
    checks=[];sources={}
    def report(p):
        r=read(p);sources[p.relative_to(OUT).as_posix()]=sha(p)
        assert r.get('devices_opened',0)==0 and r.get('new_SAC_training_steps',0)==0 and not r.get('hardware_execution_allowed',False)
        if 'source_sha256' in r:assert sha(OUT/r['source'])==r['source_sha256']
        return r
    bounds=np.load(BASE/'robust_cycle_preparation_20261111/command_contract.npz',allow_pickle=False)
    def commands(p,length):
        z=np.load(p,allow_pickle=False);values=np.vstack([z['start'],z['goals']]);assert values.shape[1]==16 and len(z['goals'])==length and np.isfinite(values).all()
        assert np.max(abs(np.diff(values,axis=0)),initial=0)<=2+1e-9 and np.all(values>=bounds['lower']) and np.all(values<=bounds['upper'])
    audit=report(FOLDER/'entry_state_comparison.json')
    for state in audit['states']:assert sha(OUT/state['path'])==state['sha256']
    a,b=audit['states'];delta=(np.asarray(b['cube_center_world_m'])-a['cube_center_world_m'])*1000
    assert np.allclose(delta,audit['desired_cube_translation_world_mm'],rtol=0,atol=1e-12)
    assert abs(np.linalg.norm(delta)-audit['cube_center_difference_mm'])<1e-12 and 9.6<audit['cube_center_difference_mm']<9.7
    assert np.max(abs(np.asarray(b['command_motor_ticks'])-a['command_motor_ticks']))==audit['maximum_motor_command_difference_ticks']==591
    visual=report(FOLDER/'visual_manifest.json');assert visual['source_report_sha256']==sha(FOLDER/'entry_state_comparison.json') and visual['same_camera']
    assert (FOLDER/'entry_comparison.png').exists()
    checks.append('Actual reached state and frozen successful reference differ by9.638mm and591ticks; comparison and same-camera views preserve source hashes')
    planners=[]
    for name in ('robust_mu025_reposition_to_nominal_entry_v1','robust_mu025_regrasp_nominal_entry_v2','robust_mu025_entry_orientation_repair_v3'):
        path=BASE/name/'review.json'
        if not path.exists():continue
        r=report(path);planners.append(name)
        assert r['mode']=='entry' and r['global_yaw_target_deg']==90 and r['sliding_friction_hypothesis']==.25
        assert r['transport_first_requested'] and r['original_native_guards_and_joint_limits_preserved'] and r['actual_state_resets_after_start']==0
        assert sha(OUT/r['goal_reference'])==r['goal_reference_sha256']
        commands(path.parent/'research_goals.npz',r['steps'])
        assert r['worst']['self_mm']<=1e-6 and r['worst']['range_rad']<=1e-6 and r['worst']['tracking_rad']<=.18
        if name.endswith('_v1'):
            assert r['final']['center_target_error_mm']<1 and r['final']['error_deg']<5
            assert r['stop']=='final_hold_upper_certificate' and not r['conditional_bridge_and_hold_passed']
        else:
            assert r['entry_command_tolerance_ticks']==8 and r['entry_hand_alignment_is_additional_task_requirement_not_guard_change']
            if r['conditional_bridge_and_hold_passed']:assert r['final']['reference_command_error_ticks']<=8
    checks.append('Position-first entry trials keep the original90degree target, native guards and2tick command bound; interrupted research-hold timing is checked separately and its upper-space failure remains recorded')
    finger=report(FOLDER/'finger_alignment_orders_v1/review.json')
    assert sha(OUT/finger['reference'])==finger['reference_sha256'] and len(finger['rows'])==10
    for r in finger['rows']:
        commands(OUT/r['path']/'research_goals.npz',r['steps'])
        assert r['actual_state_resets_after_source']==0
        if r['case']=='only_MF':assert r['stop']==['self_mm']
        if r['case']=='only_IF':assert r['final']['orientation_error_to_entry_deg']>9
        if r['case']=='RF_IF_MF_TH':
            assert not r['stop'] and r['final']['command_error_ticks']==0 and r['steps']==540
            assert 9<r['final']['orientation_error_to_entry_deg']<10
    checks.append('Four single-finger and six ordered trials retain actual support/geometry outcomes; a540goal joint-aligned arrival is not misreported as an orientation-aligned entry')
    trial_count=0
    for path in sorted((FOLDER/'suffix_trials').glob('*/review.json')):
        r=report(path);trial_count+=1
        assert sha(OUT/r['reference'])==r['reference_sha256'] and sha(OUT/r['suffix'])==r['suffix_sha256']
        commands(path.parent/'research_goals.npz',r['steps'])
        assert r['actual_state_resets_after_reached_entry']==0 and r['original_first_hold_failure_not_reclassified']
        assert r['full_initial_placement_two_turn_cycle_passed'] is False
        assert r['original_native_guards_and_joint_limits_preserved']
        if r.get('initialized_reference_control_not_low_friction_arrival'):
            assert (OUT/r['source']).resolve()==(BASE/'initial_to_reconfigured_first90/end_state.npz').resolve()
            assert r['before']['cube_center_error_to_entry_mm']==0 and r['before']['cube_orientation_error_to_entry_deg']<1e-5
        if path.parent.name=='direct_reconfigured_continued':assert 87<r['final']['final180_error_deg']<88 and not r['second_rotation_and_clear_hold_passed']
        if path.parent.name=='repositioned_then_aligned':
            assert r['before']['cube_center_error_to_entry_mm']<1 and r['after_alignment']['cube_center_error_to_entry_mm']<2
            assert r['after_alignment']['cube_orientation_error_to_entry_deg']>12 and not r['second_rotation_and_clear_hold_passed']
        if r['second_rotation_and_clear_hold_passed']:
            assert r['hold']['samples']==r['hold']['certified']==4050 and r['final']['final180_error_deg']<5 and not r['stop']
    assert trial_count>=2
    checks.append('The same archived second-turn suffix was actually replayed from reached entries; mismatched first-hold tasks and failed turns are not relabeled as successes')
    direction=report(FOLDER/'actual_contact_direction_audit/review.json')
    assert direction['static_positive_lever_not_substituted_for_actual_arrival'] and -27<direction['rows'][0]['positive_yaw_lever_mm']<-26
    assert all(r['actual_contact_count']>0 and r['positive_yaw_lever_mm']<0 for r in direction['rows'])
    blocked=report(BASE/'robust_mu025_reposition_pressure_direction_guard_v2/review.json')
    for row in blocked['rows']:
        assert row['steps']==0 and row['stop']==['actual_contact_would_push_reverse_yaw']
        commands(OUT/row['path']/'research_goals.npz',0)
    regression=report(BASE/'directed_pressure_guard_nominal_regression/review.json')
    for row in regression['rows']:
        commands(OUT/row['path']/'research_goals.npz',10);assert not row['stop']
        a=np.load(OUT/row['path']/'research_goals.npz',allow_pickle=False)['goals']
        b=np.load(BASE/'actual_A_next_MF_yaw_pressure'/f"force{row['pressure_hypothesis_N']:g}"/'research_goals.npz',allow_pickle=False)['goals'][:10]
        assert np.array_equal(a,b)
    checks.append('Actual reverse-yaw contact is rejected before pressure output; three nominal10-goal command prefixes exactly match saved prior results')
    endpoint=report(FOLDER/'standoff_endpoint_audit/review.json')
    for row in endpoint['rows']:
        assert sha(OUT/row['source'])==row['source_sha256']
        assert abs(row['native_gap_mm']-row['independent_convex_axis_gap_mm'])<1e-10
    replay=report(FOLDER/'free_standoff_replay_verification/review.json')
    assert sha(OUT/replay['parent_goals'])==replay['parent_goals_sha256']
    assert sha(BASE/'robust_mu025_free_MF_standoff_route_v1/review.json')==replay['parent_review_sha256']
    assert replay['steps']==240 and replay['all1ms_samples']==replay['supported_samples']==18000
    assert max(replay['bound_state_errors'].values())==0 and replay['no_MF_cube_penetration_contacts'] and replay['actual_state_resets_after_source']==0
    assert replay['actual_tip_contact']['gap_mm']>7 and replay['actual_tip_contact']['lever_mm']>20
    checks.append('The240-goal free approach independently reproduces all bound state fields and18000 supported samples; refreshed endpoint geometry agrees with an independent convex-axis calculation')
    latest=[]
    for name in ('robust_mu025_declared_face_feedback_v4','robust_mu025_declared_face_feedback_v5','robust_mu025_feedback_gait_push_v11'):
        study=report(BASE/name/'review.json');assert study['sliding_friction_hypothesis']==.25 and study['only_MF_commands_changed'] and study['model_force_not_calibrated_to450mA']
        assert sha(OUT/study['geometry'])==study['geometry_sha256'] and study['original_native_guards_and_joint_limits_preserved']
        for row in study['rows']:
            commands(OUT/row['path']/'research_goals.npz',row['steps'])
            assert row['hold_samples']==0 and not row['rotation_and_clear_hold_at_reached_location_passed']
            assert row['worst']['self_mm']<=1e-6 and row['worst']['support_mm']<=.15 and row['worst']['cube_mm']<=1 and row['worst']['tracking_rad']<=.18 and row['worst']['range_rad']<=1e-6
            loaded=[t for t in row['trace'] if t['actual_contact']['normal_N']>.05]
            assert loaded and all(t['actual_contact']['lever_mm']>0 for t in loaded)
            assert row['final180_error_deg']>5
            latest.append((name,row))
        if name.endswith('_v5'):assert study['rows'][0]['candidate_rejections']=={'range_rad':3} and study['rows'][0]['stop']==['no_valid_declared_face_step']
        if name.endswith('_v11'):assert not study['rows'][0]['stop'] and 37<study['rows'][0]['final180_error_deg']<38 and study['rows'][0]['steps']==1600
    reset=report(BASE/'robust_mu025_feedback_reset_v7/review.json');assert reset['abduction_reset_deg']==-10 and reset['other12_joint_commands_bitwise_unchanged']
    selected=next(r for r in reset['rows'] if r['label']=='pip-15_dip0_mcp0');assert selected['whole_hand_supported_MF_reset_passed'] and selected['samples']==2250 and selected['whole_hand_support_fraction']==1 and selected['cube_orientation_change_deg']<3
    route=report(BASE/'robust_mu025_feedback_free_route_v10/review.json');commands(BASE/'robust_mu025_feedback_free_route_v10/research_goals.npz',110)
    assert not route['stop'] and route['actual_standoff_reached'] and route['whole_hand_support_fraction']==1
    assert route['model_collision_masks_and_explicit_pairs_unchanged'] and route['planning_clearance_mm']>.0
    assert set(route['cube_collision_geoms_checked'])=={'mf_bs_collision_1','mf_px_collision','mf_md_collision_1','mf_md_collision_5','mf_ds_collision_1','mf_tip'}
    checks.append('Declared-face feedback makes positive-yaw progress to37.578degrees error; joint-limit candidate rejection, supported abduction reset and110-goal free approach retain original native limits and uncompleted final holds')
    pending=report(FOLDER/'continuation.json');assert not pending['all_segment_durations_75ms'] and pending['timing_is_saved_timestamps_not_dynamic_replay']
    exceptions=[r for r in pending['segment_timing'] if r['final_command_duration_ms']!=75]
    assert len(exceptions)==1 and exceptions[0]['final_command_duration_ms']==38 and sum(r['goals'] for r in pending['segment_timing'])==6736
    for row in pending['segment_timing']:assert sha(OUT/row['path'])==row['sha256']
    for job in pending['pending_jobs']:
        assert sha(OUT/job['script'])==job['script_sha256'] and job['execution_requires_working_approval_review'] and job['completed']
        assert sha(BASE/job['expected_report'])==job['result_sha256']
        if 'executed_source_snapshot' in job:assert sha(OUT/job['executed_source_snapshot'])==job['executed_source_sha256']
    assert not pending['approval_usage_block_current']
    for name,digest in pending['completed_reports_sha256'].items():assert sha(OUT/name)==digest
    exact=report(BASE/'robust_mu025_feedback_chain_exact_replay_v1/review.json');regular=report(BASE/'robust_mu025_feedback_chain75ms_replay_v1/review.json')
    for r in (exact,regular):
        assert not r['stop'] and r['steps']==6736 and r['actual_state_resets_after_initial_placement']==0 and not r['full_two_turn_two_hold_success']
        assert r['first_hold']['samples']==r['first_hold']['supported_samples']==r['first_hold']['upper_certified_samples']==4050
        assert r['first_hold']['minimum_upper_lower_mm']>1 and r['worst']['self_mm']<=1e-6 and r['worst']['range_rad']<=1e-6
        for name,digest in r['source_files_sha256'].items():assert sha(OUT/name)==digest
    assert exact['all1ms_samples']==505163 and exact['short_interrupted_research_commands']==1
    assert len(exact['seams'])==11 and all(max(s['bound_state_errors'].values())==0 for s in exact['seams'])
    assert regular['all1ms_samples']==505200 and regular['short_interrupted_research_commands']==0 and regular['original75ms_schedule_used']
    assert abs(regular['final180_error_deg']-exact['final180_error_deg'])<.02
    commands(BASE/'robust_mu025_feedback_chain75ms_replay_v1/research_goals.npz',6736)
    checks.append('After approval access resumed, all6736 commands were replayed with11 exact state seams; completing the interrupted38ms command to75ms also passes native guards and first4050-sample hold, with a still-incomplete37.591degree final error')
    for name in ('robust_mu025_feedback_pressure_v12','robust_mu025_feedback_bounded_fit_v13','robust_mu025_feedback_net_moment_v14'):
        r=report(BASE/name/'review.json')
        for row in r['rows']:
            commands(OUT/row['path']/'research_goals.npz',row['steps'])
            assert row['hold_samples']==0 and not row['rotation_and_clear_hold_at_reached_location_passed']
            assert row['worst']['self_mm']<=1e-6 and row['worst']['range_rad']<=1e-6
        if name.endswith('_v12'):assert 26<r['rows'][0]['final180_error_deg']<27 and not r['rows'][0]['stop']
        if name.endswith('_v13'):assert r['rows'][0]['steps']==31 and r['rows'][0]['stop']==['no_valid_declared_face_step']
        if name.endswith('_v14'):
            assert r['net_finger_direction'] and r['bounded_joint_fit'] and not r['rows'][0]['stop']
            assert 25<r['rows'][0]['final180_error_deg']<26 and any(t['finger_moment']['strict_any_opposed_rejected'] and not t['finger_moment']['net_direction_rejected'] for t in r['rows'][0]['trace'])
    moment=report(FOLDER/'finger_moment_admission_audit/review.json');assert not moment['unchanged_command_75ms_native_stop']
    opposed=[r for r in moment['rows'] if r['strict_any_opposed_rejected']];assert opposed
    for r in opposed:
        assert r['normal_yaw_moment_Nm']>0 and r['total_yaw_moment_Nm']>0 and not r['net_direction_rejected']
        assert abs(sum(c['normal_yaw_moment_Nm'] for c in r['contacts'])-r['normal_yaw_moment_Nm'])<1e-12
    failure=report(BASE/'robust_mu025_TH_relief_v18/review.json');commands(BASE/'robust_mu025_TH_relief_v18/research_goals.npz',failure['steps'])
    assert failure['steps']==993 and failure['stop']==['no_valid_two_finger_step'] and failure['candidate_rejections']=={'cube_translation_over80mm':3}
    assert failure['final180_error_deg']>120 and failure['tilt_deg']>60 and not failure['rotation_and_clear_hold_at_reached_location_passed']
    checks.append('Contact-moment sums identify overstrict individual-contact rejection; bounded/net-direction studies remain incomplete and the later thumb-unloading fall is preserved as failure')
    release=report(BASE/'robust_mu025_late_TH_normal_release_v35/review.json');assert release['actual_normal_release'] and release['release_gap_mm']==12 and release['only_declared_finger_commands_changed']=='th'
    rr=release['rows'][0];commands(OUT/rr['path']/'research_goals.npz',775)
    assert not rr['stop'] and rr['hold_samples']==rr['supported_samples']==4050 and rr['upper_certified_samples']==0
    assert rr['final180_error_deg']<3 and rr['tilt_deg']<2 and not rr['rotation_and_clear_hold_at_reached_location_passed']
    arrival=read(OUT/rr['path']/'rotation_arrival_review.json');assert arrival['error180_deg']<3 and arrival['tilt_deg']<5 and arrival['rotation_only_not_final_hold']
    old_release=report(BASE/'robust_mu025_late_TH_feedback_v34/review.json')
    old_commands=[np.load(OUT/r['path']/'research_goals.npz',allow_pickle=False)['goals'] for r in old_release['rows']]
    assert np.array_equal(old_commands[0],old_commands[1]),'Pressure hypotheses saturated to identical commands; do not claim separate effectiveness'
    assert np.array_equal(old_commands[0][:arrival['steps']],np.load(OUT/rr['path']/'rotation_arrival_goals.npz',allow_pickle=False)['goals'])
    space=report(FOLDER/'final_hold_space_audit_named/review.json');assert set(space['certificate_bad_geoms'])=={'cadtip_mf_th_mf_bs_0','cadtip_mf_rf_mf_bs_0','mf_bs_visual_support_proxy'}
    final_comparisons=report(BASE/'robust_mu025_final_clear_hold_v37/review.json');assert len(final_comparisons['rows'])==4
    for r in final_comparisons['rows']:
        commands(OUT/r['path']/'research_goals.npz',r['steps']);assert not r['rotation_and_clear_hold_at_reached_location_passed']
        if r['label']=='known_clear_hand':assert r['upper_certified_samples']==4050 and r['final180_error_deg']>17
    final=report(BASE/'robust_mu025_final_pose_space_v38/review.json');assert final['conditional_bridge_and_hold_passed'] and final['stop']=='conditional_task_hold_passed'
    assert final['global_yaw_target_deg']==180 and final['upper_planning_objective_weight']==3 and final['upper_certificate_threshold_unchanged']
    commands(BASE/'robust_mu025_final_pose_space_v38/research_goals.npz',86);window=final['final_hold']
    assert window['samples']==window['certified_samples']==4050 and window['support_fraction']==1 and .3<window['min_lower_mm']<.4 and window['final_error_deg']<5
    cycle=report(BASE/'robust_mu025_complete_cycle75ms_replay_v39/review.json');commands(BASE/'robust_mu025_complete_cycle75ms_replay_v39/research_goals.npz',9614)
    assert not cycle['stop'] and cycle['full_two_turn_two_hold_success'] and cycle['all1ms_samples']==721050 and cycle['original75ms_schedule_used']
    assert cycle['actual_state_resets_after_initial_placement']==0 and len(cycle['seams'])==18 and not cycle['same_first_station_2mm_cycle_passed']
    for h in (cycle['first_hold'],cycle['final_hold']):
        assert h['samples']==h['supported_samples']==h['upper_certified_samples']==4050 and h['minimum_upper_lower_mm']>.3 and h['orientation_error_deg']<5 and h['tilt_deg']<5 and h['shift_mm']<2 and h['drift_deg']<3
    assert 20<cycle['first_station_error_mm']<21 and 4<cycle['final180_error_deg']<5
    for name,digest in cycle['source_files_sha256'].items():assert sha(OUT/name)==digest
    handoff=report(FOLDER/'complete_cycle_handoff.json');assert handoff['archived_learned_policy_derived_prefix_goals']==1700 and handoff['remaining_privileged_model_teacher_goals']==7914
    assert not handoff['new_distillation_completed'] and .0<handoff['first_hold_direction_margin_to5deg']<.02 and 0<handoff['final_upper_margin_above0p3mm']<.04
    for name,digest in handoff['source_files_sha256'].items():assert sha(OUT/name)==digest
    for case in handoff['finite_variations']:
        assert case['report_available'] and not case['full_two_turn_two_hold_success'] and case['stop']==['range_rad'] and case['steps']==7744
        r=report(BASE/('complete_cycle_variation_'+case['case'])/'review.json');commands(BASE/('complete_cycle_variation_'+case['case'])/'research_goals.npz',7744)
        assert r['actual_state_resets_after_initial_placement']==0 and r['all1ms_samples']==580743 and r['worst']['range_rad']>1e-6
        if case['case']=='xplus1':assert case['first_hold']['orientation_error_deg']>5
    regression=report(BASE/'unperturbed_cycle_prefix_regression/regression.json');assert regression['all_qpos_array_values_exactly_equal'] and regression['steps']==3669
    assert sha(OUT/regression['reference'])==regression['reference_sha256']
    plot=report(FOLDER/'late_rotation_plot.json');assert plot['conditional_final_hold_passed'] and plot['full_initial_placement_replay_reported_separately']
    for name,digest in plot['sources_sha256'].items():assert sha(OUT/name)==digest
    assert (FOLDER/'late_rotation_progress.png').exists() and (FOLDER/'late_rotation_progress.svg').exists()
    checks.append('New actual-normal TH release preserves rotation; final joint/position/space correction and9614-goal75ms full replay certify both4050-sample holds only in the specified model condition, while all three finite perturbations fail and the unperturbed3669-goal implementation regression is exact')
    published=report(FOLDER/'review.json')
    for name,digest in published['source_reports_sha256'].items():assert sha(OUT/name)==digest
    assert published['old_nominal_two_turn_success_preserved'] and published['original2mm_same_station_miss_not_added_as_prerequisite']
    assert not published['all_segment_durations_75ms'] and published['full_initial_placement_chain_replay_completed'] and published['complete75ms_partial_chain_replay_completed']
    assert not published['pending_model_calculations_due_to_approval_usage_limit'] and published['full_two_turn_two_hold_task_passed'] and not published['new_cycle_small_error_robustness_passed']
    readiness=read(OUT/'next_cycle/readiness.json')['rotation_entry_reposition'];assert readiness['sha256']==sha(FOLDER/'review.json') and not readiness['hardware_execution_allowed']
    result=dict(status='passed',checks=checks,planners=planners,suffix_trials=trial_count,source_reports_sha256=sources,verification_is_saved_model_evidence_not_physical_success=True,devices_opened=0,hardware_execution_allowed=False)
    (FOLDER/'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8');return result


if __name__=='__main__':print(json.dumps(verify(),ensure_ascii=False),flush=True)
