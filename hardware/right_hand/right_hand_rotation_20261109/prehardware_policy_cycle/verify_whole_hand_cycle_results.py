"""Audit continuous replay, rate bounds, support transitions and retained failures.

This audits saved evidence. It does not run dynamics, train SAC or authorize
physical motion. Finger load comparisons are kept separate from whole support.
"""
from pathlib import Path
import hashlib
import json
import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
BASE=OUT/'next_cycle/handoff_refinement_20261110'
BOUND=('qpos','qvel','ctrl','act','qacc_warmstart')
SIM_IDS=np.array((8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1))-1


def read(p):return json.loads(p.read_text(encoding='utf-8'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def verify():
    checks=[];audited={};command_files={}
    def report(name):
        p=BASE/name/'review.json';r=read(p);audited[p.relative_to(OUT).as_posix()]=sha(p)
        assert r.get('devices_opened',0)==0 and r.get('new_SAC_training_steps',0)==0
        assert not r.get('hardware_execution_allowed',False)
        if 'source' in r and 'source_sha256' in r:assert sha(OUT/r['source'])==r['source_sha256'],name
        for key in ('source_files_sha256','source_reports_sha256'):
            for n,digest in r.get(key,{}).items():assert sha(OUT/n)==digest,(name,n)
        prov=r.get('model_provenance',r.get('source_model_provenance'))
        if prov:
            assert sha(ROOT/prov['model_path'])==prov['model_sha256']
            assert prov['max_goal_ticks']==2 and prov['control_seconds']==.075
            assert prov['native_geometry_limits_and_guard_thresholds_preserved']
        return r
    def commands(folder,steps=None,fixed_sim=(),fixed_motor=(),filename='research_goals.npz'):
        p=folder/filename;z=np.load(p,allow_pickle=False);goals=z['goals'];start=z['start']
        assert goals.ndim==2 and goals.shape[1]==16 and np.isfinite(goals).all()
        if steps is not None:assert len(goals)==steps,(p,len(goals),steps)
        assert np.max(abs(np.diff(np.vstack([start,goals]),axis=0)))<=2+1e-9,p
        ids=np.r_[SIM_IDS[np.asarray(fixed_sim,dtype=int)],np.asarray(fixed_motor,dtype=int)].astype(int)
        if len(ids):assert np.array_equal(goals[:,ids],np.broadcast_to(start[ids],(len(goals),len(ids)))),p
        command_files[p.relative_to(OUT).as_posix()]=sha(p)
        return goals
    def bound_equal(a,b):
        a=np.load(a,allow_pickle=False);b=np.load(b,allow_pickle=False)
        assert all(np.array_equal(a[k],b[k]) for k in BOUND)
    def clean(r):
        for key,bound in (('self_mm',1e-6),('support_mm',.15),('cube_mm',1),('tracking_rad',.18),('range_rad',1e-6)):
            assert r['worst'][key]<=bound,(key,r['worst'][key])

    first=report('initial_to_reconfigured_first90')
    assert first['steps']==3405 and first['first90_reconfigured_whole_hand_hold_passed']
    assert first['actual_state_resets_after_initial_placement']==0
    assert len(first['seams'])==3 and all(max(s['bound_state_errors'].values())==0 for s in first['seams'])
    assert first['hold_samples']==first['upper_certified_samples']==4050
    assert first['minimum_upper_lower_mm']>.3 and first['whole_hand_support_fraction']==1
    assert first['final90_error_deg']<5 and first['reference_A_center_error_mm']<2
    assert first['finger_identity_is_not_a_global_support_requirement'] and first['old_canonical_task_not_reclassified']
    commands(BASE/'initial_to_reconfigured_first90',3405);clean(first)
    checks.append('Initial3405-goal first quarter has three exact complete-state seams,4050 supported and clear samples; original canonical miss and next-turn scope remain explicit')

    transfer=report('whole_hand_support_transfer_20261111')
    assert len(transfer['rows'])==6 and sum(r['intermediate_whole_hand_supported'] for r in transfer['rows'])==5
    assert transfer['other15_joint_commands_bitwise_unchanged'] and transfer['finger_contacts_may_change']
    assert 'cube_translation_over80mm' in transfer['rows'][0]['stop']
    for r in transfer['rows']:
        commands(OUT/r['path'],r['steps'],fixed_motor=[i for i in range(16) if i!=7])
        if r['intermediate_whole_hand_supported']:
            assert not r['stop'] and r['samples']==2250 and r['whole_hand_support_fraction']==1;clean(r)
    relay=report('whole_hand_contact_relay_20261111')
    assert len(relay['rows'])==2 and relay['whole_hand_support_includes_palm'] and relay['contact_identity_may_change']
    for r in relay['rows']:
        commands(OUT/r['path'],r['steps']);clean(r)
        assert r['intermediate_contact_relay_passed'] and not r['stop']
        assert r['relay_hold_samples']==4050 and r['whole_hand_hold_support_fraction']==1
        assert r['final90_error_deg']>5 and r['final_MF_TH_normals_N']['th']==0
    checks.append('Six root-only transfers and two relay trials retain one model object-loss failure; palm support and contact changes are valid but intermediate tilts are not final-grasp success')

    actor=report('actual_first_A_original108_reuse')
    assert not actor['source_has_hypothetical_initialized_ancestor']
    assert actor['source_initial_prefix_replay']['exact_bound_state_match']
    assert len(actor['rows'])==2 and all(not r['next_rotation_and_hold_passed'] and 'self_mm' in r['stop'] for r in actor['rows'])
    for name in ('first90_loaded_MFTH_entry1000','first90_loaded_MFTH_entry1700','first90_loaded_cp152_retained_hybrid','actual_first_A_reconfigured_next90','actual_first_A_support_transfer_next90'):
        r=report(name);commands(BASE/name,r['steps']);clean(r)
        assert not r['conditional_bridge_and_hold_passed'] and r['actual_state_resets_after_start']==0
        assert r['original_native_guards_and_joint_limits_preserved']
        assert r.get('retain_loaded_contacts_at_control_boundaries',False)==(name=='first90_loaded_cp152_retained_hybrid')
    checks.append('Actual-start108-actor failures and five finite teacher comparisons remain failed; sustained MF/TH loads were imposed only in the declared comparative branch')

    reset=report('actual_A_next_MF_reset376')
    assert reset['other12_joint_commands_bitwise_unchanged'] and reset['specific_MF_contact_not_required']
    assert len(reset['rows'])==6
    for r in reset['rows']:
        commands(OUT/r['path'],r['steps'],fixed_sim=np.r_[0:4,8:16]);clean(r)
        assert r['whole_hand_supported_MF_reset_passed'] and not r['stop']
        assert r['samples']==2250 and r['whole_hand_support_fraction']==1
        assert r['MF_PIP_upper_room_deg']>20 and r['cube_shift_mm']<.22 and r['cube_orientation_change_deg']<.22
    checks.append('Six MF repositions preserve the other12 controls and2250 whole-hand support samples while obtaining joint room; they are not quarter-turn completions')

    geometry_names=('actual_A_next_MF_positive_yaw_geometry','actual_A_next_MF_positive_yaw_any_height','actual_A_next_counter_thumb_geometry','actual_A_next_counter_ring_geometry')
    for name in geometry_names:
        r=report(name)
        assert len(r['rows'])==4 and r['cube_fixed_only_in_proposal_copies'] and r['free_object_arrival_not_evaluated']
        for row in r['rows']:
            assert sha(OUT/row['proposal'])==row['proposal_sha256']
            if row['static_contact_candidate']:
                assert any(c['positive_yaw_moment_arm_mm']>3 and c['normal_alignment']>.7 for c in row['contacts'])
                assert max(row['self_pairs_mm'].values(),default=0)<=1e-6 and row['cube_penetration_mm']<=1
    for name,expected_passes in (('actual_A_next_MF_positive_yaw_arrival',3),('actual_A_next_counter_thumb_arrival',3)):
        r=report(name)
        assert r['all_actual_cube_states_free'] and r['specific_contact_not_required_to_persist']
        assert sha(OUT/r['geometry'])==r['geometry_sha256']
        assert sum(not row['stop'] for row in r['rows'])==expected_passes
        for row in r['rows']:
            if 'path' not in row:continue
            f=row['finger'];active=np.arange(4*('if','mf','rf','th').index(f),4*('if','mf','rf','th').index(f)+4)
            commands(OUT/row['path'],row['steps'],fixed_sim=np.setdiff1d(np.arange(16),active))
            assert not row['full_next90_and_hold_passed']
            if not row['stop']:
                assert row['hold_samples']==4050 and row['whole_hand_support_fraction']==1
                assert abs(row['yaw_advance_deg'])<1;clean(row)
            else:assert 'cube_translation_over80mm' in row['stop']
    checks.append('Fixed-object IK hypotheses are separated from free-object arrivals; three thumb arrivals preserve support, three other arrivals lose the object, and no RF candidate in this limited tip search is a general reachability proof')

    pressure=report('actual_A_next_MF_yaw_pressure');all_goals=[]
    assert len(pressure['rows'])==3 and pressure['contact_identity_not_a_global_invariant']
    for r in pressure['rows']:
        goals=commands(OUT/r['path'],500,fixed_sim=np.r_[0:4,8:16]);all_goals.append(goals);clean(r)
        assert not r['stop'] and 15<r['yaw_advance_deg']<16 and 30<r['final180_error_deg']<31
        assert r['hold_samples']==0 and not r['rotation_and_whole_hand_hold_passed']
        assert not r['reference_A_station_2mm_passed'] and r['final_pose_upper_bad_geoms']
    assert all(np.array_equal(all_goals[0],g) for g in all_goals[1:])
    continued=report('actual_A_next_MF_yaw_pressure_continued');r=continued['rows'][0]
    goals=commands(OUT/r['path'],r['steps'],fixed_sim=np.r_[0:4,8:16])
    assert r['steps']==753 and np.array_equal(goals[:500],all_goals[0])
    proof=r['verified_prior_prefix'];assert proof['goals_equal'] and proof['physical_state_not_reset']
    assert max(proof['bound_state_errors'].values())==0
    assert 'cube_translation_over80mm' in r['stop'] and r['best180_error_deg']<20
    assert r['final180_error_deg']>80 and not r['rotation_and_whole_hand_hold_passed']
    checks.append('Three pressure settings produced the same500-command path, not three independent successes; a753-command extension exactly matches that prefix but loses the model object after a transient19.85-degree error')

    chain=report('initial_through_next_MF_contact_gait')
    assert chain['steps']==4673 and len(chain['seams'])==5
    assert all(max(r['bound_state_errors'].values())==0 for r in chain['seams'])
    assert chain['actual_state_resets_after_initial_placement']==0
    assert chain['all1ms_samples']==4673*75 and chain['whole_hand_support_fraction']>.999
    assert chain['first_hold_samples']==chain['first_hold_upper_certified_samples']==4050
    assert 61<chain['additional_next_yaw_deg']<63 and 30<chain['final180_error_deg']<31
    assert chain['reference_first_station_error_mm']>7 and chain['final_pose_upper_bad_geoms']
    assert chain['partial_second_turn_not_full90'] and not chain['full_initial_two_positive_turn_cycle_passed']
    goals=commands(BASE/'initial_through_next_MF_contact_gait',4673);clean(chain)
    first_goals=np.load(BASE/'initial_to_reconfigured_first90/research_goals.npz',allow_pickle=False)['goals']
    assert np.array_equal(goals[:3405],first_goals) and np.array_equal(goals[-500:],all_goals[0])
    bound_equal(BASE/'initial_through_next_MF_contact_gait/end_state.npz',BASE/'actual_A_next_MF_yaw_pressure/force0.5/end_state.npz')
    checks.append('4673 goals independently replay from the original initial state with five exact complete-state seams; the second turn remains partial and final top clearance and2mm station checks remain failed')

    teacher_path=BASE/'actual_A_next_two_contact_centered_teacher/review.json'
    if teacher_path.exists():
        teacher=report('actual_A_next_two_contact_centered_teacher')
        commands(teacher_path.parent,teacher['steps']);clean(teacher)
        assert teacher['actual_state_resets_after_start']==0 and teacher['model_based_teacher_not_SAC']
        assert not teacher['retain_loaded_contacts_at_control_boundaries'] and teacher['original_native_guards_and_joint_limits_preserved']
        checks.append('Two-contact centered teacher retains its exact source, no contact-identity invariant and the unchanged physical guards; its task outcome is read from the saved report')

    hold_path=BASE/'actual_A_second_turn_clear_hold/review.json'
    if hold_path.exists():
        holds=report('actual_A_second_turn_clear_hold')
        assert holds['contact_identity_not_required'] and holds['first_station_target_and2mm_threshold_unchanged']
        assert len(holds['rows'])==4
        for row in holds['rows']:
            commands(OUT/row['path'],row['steps'],fixed_sim=np.setdiff1d(np.arange(16),row['changed_sim_indices']))
            assert not row['same_first_station_next_hold_passed']
        best=next(r for r in holds['rows'] if r['label']=='unchanged');clean(best)
        assert not best['stop'] and best['steps']==54 and best['rotation_and_clear_hold_at_reached_location_passed']
        assert best['hold_samples']==best['upper_certified_samples']==4050 and best['whole_hand_hold_support_fraction']==1
        assert best['final180_error_deg']<5 and best['tilt_deg']<5 and best['upper_minimum_lower_mm']>1
        assert 7<best['first_station_error_mm']<8 and best['hold_shift_mm']<.2 and best['hold_drift_deg']<.2
        assert all(row['stop'] for row in holds['rows'][1:])
        checks.append('Reached second-turn location has4050 clear and supported samples without changing commands; three posture-return failures and the7.4mm station miss remain separate')

    complete_path=BASE/'initial_second_turn_hold_replay/review.json'
    if complete_path.exists():
        complete=report('initial_second_turn_hold_replay');clean(complete)
        assert complete['steps']==5089 and complete['actual_state_resets_after_initial_placement']==0
        assert len(complete['seams'])==4 and all(max(s['bound_state_errors'].values())==0 for s in complete['seams'])
        assert complete['all1ms_samples']==5089*75 and complete['whole_hand_supported_fraction']>.999
        assert complete['two_rotations_and_clear_holds_at_reached_locations_passed'] and not complete['same_first_station_two_turn_cycle_passed']
        assert 91<complete['additional_yaw_from_first_hold_deg']<92 and complete['final180_error_deg']<5 and complete['first_station_error_mm']>7
        assert len(complete['holds'])==2
        for h in complete['holds']:
            assert h['samples']==h['supported_samples']==h['upper_certified_samples']==4050
            assert h['minimum_upper_lower_mm']>.3 and h['hold_shift_mm']<2 and h['hold_drift_deg']<3
        all_commands=commands(complete_path.parent,5089)
        assert np.array_equal(all_commands[:4673],goals)
        bound_equal(complete_path.parent/'end_state.npz',BASE/'actual_A_second_turn_clear_hold/unchanged/end_state.npz')
        checks.append('5089-goal original-initial replay has four exact seams and both full4050-state support/space holds; nominal reached-location success is distinguished from the failed2mm common station and hardware readiness')

    perturb_path=BASE/'initial_two_turn_finite_perturbations/review.json'
    if perturb_path.exists():
        perturb=report('initial_two_turn_finite_perturbations')
        assert perturb['actual_state_resets_after_initial_placement']==0 and perturb['nominal_path_not_reoptimized']
        assert perturb['saved_path_not_a_new_closed_loop_SAC_policy'] and perturb['friction_is_hypothesis_not_measurement']
        assert len(perturb['rows'])==3 and sum(r['two_rotation_clear_holds_at_reached_locations_passed'] for r in perturb['rows'])==1
        for r in perturb['rows']:
            assert r['total_goals']==5089 and not r['same_first_station_cycle_passed']
            if r['two_rotation_clear_holds_at_reached_locations_passed']:
                clean(r);assert r['case']=='initial_MF_TH_one_tick' and not r['stop']
                assert r['executed_goals']==5089 and r['hold_samples']==r['hold_supported_samples']==r['hold_upper_certified_samples']==[4050,4050]
            else:assert r['stop'] and r['executed_goals']<3405
        checks.append('One-tick initial joint perturbation passes both reached-location holds;1mm placement and assumed mu0.25 fail before first hold, so nominal replay is not a robustness certificate')

    recenter_path=BASE/'second_hold_IF_support_recentering/review.json'
    if recenter_path.exists():
        recenter=report('second_hold_IF_support_recentering');assert recenter['only_IF_commands_changed'] and recenter['all_actual_cube_states_free']
        assert len(recenter['rows'])==4
        for r in recenter['rows']:
            commands(OUT/r['path'],r['steps'],fixed_sim=np.arange(4,16));clean(r)
            assert not r['stop'] and r['hold_samples']==r['upper_certified_samples']==4050 and r['whole_hand_hold_support_fraction']==1
            assert r['first_station_error_mm']>6 and not r['same_first_station_hold_passed']
        assert sum(r['rotation_and_clear_hold_passed'] for r in recenter['rows'])==1
        checks.append('Four IF support-translation probes preserve the other12 controls and whole-hand/space holds, but none returns within2mm; linear support Jacobians are not mislabeled as object motion proof')

    published_path=BASE/'first_to_next_contact_bridge_20261111/review.json'
    if published_path.exists():
        published=read(published_path)
        for n,digest in published['source_reports_sha256'].items():assert sha(OUT/n)==digest,(n,'Report changed after publication')
        assert published['specific_finger_contact_not_a_global_invariant'] and published['palm_and_all_hand_parts_count_as_support']
        assert not published['full_initial_two_positive_quarter_cycle_passed'] and not published['hardware_execution_allowed']
    result=dict(status='passed',checks=checks,source_reports_sha256=audited,command_files_sha256=command_files,
        pressure_settings_with_identical_command_path=3,verification_is_saved_evidence_audit_not_new_dynamics=True,
        nominal_two_turn_reached_locations_passed=complete_path.exists() and complete['two_rotations_and_clear_holds_at_reached_locations_passed'],
        full_two_turn_cycle_passed=False,full_cycle_scope='Original common station and required robustness, not nominal reached-location holds',new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False)
    (BASE/'first_to_next_contact_bridge_20261111/verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    return result


if __name__=='__main__':print(json.dumps(verify(),ensure_ascii=False),flush=True)
