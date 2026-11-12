"""Audit sources, native command spacing, replay seams and distinct task scopes."""
from pathlib import Path
import json
import hashlib
import numpy as np

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1];BASE=OUT/'next_cycle/handoff_refinement_20261110'
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def verify():
    checks=[];photo=read(BASE/'user_ring_photo_reference_20261110/review.json')
    assert photo['user_contact_clarification']['ring_contact_face']=='white_front_face'
    assert sha(BASE/'user_ring_photo_reference_20261110/user_photo.jpg')==photo['photo_sha256']
    geom=read(BASE/'ring_thumb_photo_prior_seed431/review.json');exact=read(BASE/'ring_thumb_photo_prior_seed432_exact_static/review.json')
    expected=sha(ROOT/'ring_reference_readonly_20261109_124719_945552/result.json')
    assert geom['recorded_ring_source_sha256']==exact['recorded_ring_source_sha256']==expected
    assert np.array_equal(geom['recorded_ring_radians'],exact['recorded_ring_radians'])
    assert np.count_nonzero(np.asarray(geom['ring_difference_from_record_deg']))==1
    assert abs(geom['ring_difference_from_record_deg'][1]+10.876953125)<1e-10
    assert exact['exact_recorded_ring_static_only'] and exact['rows'][0]['static_clear_opposed_grip_candidate'] and not exact['rows'][0]['native_goal_admissible']
    assert geom['rows'][0]['static_clear_opposed_grip_candidate'] and geom['old_limits_and_collision_geometry_not_changed']
    assert not geom['arrival_path_passed'] and not geom['free_object_hold_passed']
    checks.append('User front-face clarification, photo and exact recorded angles are preserved; native reference differs only at ID14 and is not mislabeled as exact photo reconstruction')
    cold=read(BASE/'ring_thumb_photo_prior_seed431/axis0_ring+1/free_hold_tests/review.json')
    original=next(r for r in cold['rows'] if r['preload_hypothesis_N']==.8)
    assert not original['initialized_hold_passed'] and original['maximum_station_drift_mm']>9
    assert original['RF_load_fraction']==original['TH_load_fraction']==1 and original['certified_upper_samples']==4050
    checks.append('Original cold-placement9mm movement remains failed, even though later free dynamics has loaded RF/TH and clear upper space')
    closing=read(BASE/'ring_thumb_only_closure/review.json');assert len(closing['rows'])==3
    for r in closing['rows']:
        assert r['thumb_loaded_pose_and_clearance_hold_passed'] and r['non_thumb_controls_bitwise_unchanged'] and r['source_has_initialized_ancestor']
        assert r['hold_samples']==r['upper_certified_samples']==4050 and r['minimum_upper_lower_mm']>.3
        assert r['cube_shift_from_source_mm']<2 and r['orientation_change_from_source_deg']<3
        z=np.load(OUT/r['path']/'research_goals.npz',allow_pickle=False)
        assert len(z['goals'])==r['steps'] and np.max(abs(np.diff(np.vstack([z['start'],z['goals']]),axis=0)))<=2+1e-9
        assert np.max(abs(z['goals'][:,4:]-z['start'][4:]))==0.
    assert closing['rows'][2]['final_TH_normal_N']>closing['rows'][0]['final_TH_normal_N']
    checks.append('Three thumb-only closures preserve every non-thumb command and2tick spacing while meeting4050-state support/space and final pose criteria')
    replay=read(BASE/'ring_thumb_closure_replay/review.json');assert replay['status']=='passed' and len(replay['seams'])==2
    assert all(max(r['errors'].values())==0 for r in replay['seams'])
    assert replay['hold_samples']==replay['upper_certified_samples']==replay['RF_loaded_samples']==replay['TH_loaded_samples']==4050
    assert replay['cold_initial_pose_and_control_assignment_explicit'] and replay['arrival_from_first_rotation_not_certified']
    for name,digest in replay['source_files_sha256'].items():assert sha(OUT/name)==digest
    checks.append('Independent cold-settle plus thumb100-goal replay has two exact bound-state seams and4050 loaded/clear samples, with initialization explicitly retained')
    loads=read(BASE/'ring_thumb_settled_load_hypotheses/review.json');assert loads['required_contact_fingers']==['rf','th']
    assert len(loads['cases'])==11 and all(r['conditional_hold_passed'] and r['certified_window_samples']==4050 for r in loads['cases'])
    assert all(set(r['required_finger_over_point1N_fractions'])=={'rf','th'} and min(r['required_finger_over_point1N_fractions'].values())>.8 for r in loads['cases'])
    assert loads['station_scope']=='source' and loads['applied_moment_is_hypothesis_not_measured_left_hand_load']
    checks.append('All11 initialized friction/load hypotheses preserve both required finger loads; modeled force is not a measured left-hand capacity')
    other=read(BASE/'resumed_next90_joined_branch/review.json');assert other['steps']==637 and other['conditional_next90_between_two_stations_passed']
    assert other['actual_state_resets_after_start']==0 and all(max(r['bound_state_errors'].values())==0 for r in other['seams'])
    assert 89<other['additional_yaw_deg']<91 and other['final180_error_deg']<5 and other['target_station_error_mm']<2
    assert other['hold_samples']==other['upper_certified_samples']==4050 and other['upper_minimum_lower_mm']>.3
    assert other['start_to_target_station_distance_mm']>19 and other['source_is_initialized_MF_TH_grip_not_photo_RF_grip']
    assert not other['same_station_cycle_passed'] and not other['full_initial_placement_two_turn_cycle_passed']
    for name,digest in other['source_files_sha256'].items():assert sha(OUT/name)==digest
    commands=np.load(BASE/'resumed_next90_joined_branch/research_goals.npz',allow_pickle=False)
    assert len(commands['goals'])==637 and np.max(abs(np.diff(np.vstack([commands['start'],commands['goals']]),axis=0)))<=2+1e-9
    checks.append('Separate637-goal90-degree replay has exact seams and full hold but two stations19.5mm apart; it is not combined with the photo grip or first rotation')
    actor=read(BASE/'ring_thumb_original_actor_reuse/review.json');assert len(actor['rows'])==2 and actor['new_SAC_training_steps']==0
    assert all(not r['next_rotation_and_hold_passed'] and 'self_mm' in r['stop'] for r in actor['rows'])
    for name in ('ring_thumb_next90','ring_thumb_relief_resume','ring_thumb_tightened_relief_next90','ring_thumb_after_release_next90','ring_thumb_after_pair_retreat_next90','ring_thumb_after_regrasp8_next90'):
        p=BASE/name/'review.json'
        if not p.exists():continue
        r=read(p);assert r['actual_state_resets_after_start']==0 and r['original_native_guards_and_joint_limits_preserved']
        assert r['new_SAC_training_steps']==0 and not r['hardware_execution_allowed']
        z=np.load(p.parent/'research_goals.npz',allow_pickle=False)
        assert len(z['goals'])==r['steps'] and np.max(abs(np.diff(np.vstack([z['start'],z['goals']]),axis=0)))<=2+1e-9
        assert sha(ROOT/r['source_model_provenance']['model_path'])==r['source_model_provenance']['model_sha256']
    checks.append('Original actor failures and completed relief trials retain their actual decisions and original model/command guards')
    clear=read(BASE/'ring_tip_palm_clearance_waypoints/review.json');release=read(BASE/'ring_thumb_release_after_tip_clear/review.json');retreat=read(BASE/'ring_thumb_pair_clear_retreat_v2/review.json')
    assert sum(r['RF_clear_supported_waypoint_passed'] for r in clear['trials'])==2
    assert sum(r['intermediate_supported'] for r in release['rows'])==2
    assert any(r.get('stop') for r in clear['trials']) and any(r.get('stop') for r in release['rows'])
    assert len(retreat['trials'])==3 and all(r['RF_clear_supported_waypoint_passed'] for r in retreat['trials'])
    assert retreat['thumb_clearance_included'] and retreat['RF14_requested_retreat_deg']==3 and retreat['intermediate_clearance_not_full_turn']
    assert all(r['orientation_change_deg']>3 for r in retreat['trials'])
    for study in (clear,release,retreat):
        assert study['original_guards_and_bounds_preserved'] and not study['hardware_execution_allowed']
        assert sha(OUT/study['source'])==study['source_sha256']
        for r in study.get('trials',study.get('rows',[])):
            if 'path' not in r:continue
            file=OUT/r['path']/'research_goals.npz'
            if not file.exists():continue
            z=np.load(file,allow_pickle=False)
            assert len(z['goals'])==r['steps'] and np.max(abs(np.diff(np.vstack([z['start'],z['goals']]),axis=0)))<=2+1e-9
            assert np.max(abs(z['goals'][:,4:12]-z['start'][4:12]))==0.
    opt=read(BASE/'ring_thumb_pair_clear_retreat_v2/optimizer_bounds.json')
    assert opt['strictly_interior'] and opt['native_and_software_bounds_not_expanded']
    assert np.all(np.asarray(opt['x0'])>opt['low']) and np.all(np.asarray(opt['x0'])<opt['high'])
    checks.append('RF/palm clearance and pressure-release successes preserve native failures; three paired retreat intermediates retain support without claiming the final3degree hold criterion')
    chain=read(BASE/'ring_thumb_release_chain_replay/review.json')
    assert chain['status']=='intermediate_chain_replay_passed' and chain['steps']==426 and len(chain['seams'])==4
    assert all(max(r['bound_state_errors'].values())==0 for r in chain['seams'])
    assert chain['samples']==chain['whole_hand_supported_samples']==426*75
    assert 3<chain['RF14_upper_room_deg']<5 and chain['final180_error_deg']>70
    assert chain['actual_state_resets_after_start']==0 and chain['source_grip_was_initialized_separately']
    assert chain['arrival_from_first90_not_certified'] and chain['final4050_upper_clearance_not_certified']
    assert not chain['full_two_turn_cycle_passed'] and not chain['hardware_execution_allowed']
    for name,digest in chain['source_files_sha256'].items():assert sha(OUT/name)==digest
    z=np.load(BASE/'ring_thumb_release_chain_replay/research_goals.npz',allow_pickle=False)
    assert len(z['goals'])==426 and np.max(abs(np.diff(np.vstack([z['start'],z['goals']]),axis=0)))<=2+1e-9
    checks.append('426-goal turn-prefix/clear/release/retreat native replay has four exact complete-state seams and31950 supported1ms states, explicitly initialized and incomplete for next90')
    further=read(BASE/'ring_thumb_further_regrasp8/review.json');long=read(BASE/'ring_thumb_regrasp8_chain_replay/review.json')
    assert further['local_planning_window_deg']==12 and further['RF14_requested_retreat_deg']==8 and further['original_guards_and_bounds_preserved']
    assert len(further['trials'])==3 and all(r['RF_clear_supported_waypoint_passed'] and not r['stop'] for r in further['trials'])
    assert long['status']=='intermediate_chain_replay_passed' and long['steps']==503 and len(long['seams'])==5
    assert all(max(r['bound_state_errors'].values())==0 for r in long['seams'])
    assert long['samples']==long['whole_hand_supported_samples']==503*75 and long['RF14_upper_room_deg']>10
    assert long['actual_state_resets_after_start']==0 and not long['full_two_turn_cycle_passed'] and not long['hardware_execution_allowed']
    for name,digest in long['source_files_sha256'].items():assert sha(OUT/name)==digest
    z=np.load(BASE/'ring_thumb_regrasp8_chain_replay/research_goals.npz',allow_pickle=False)
    prior=np.load(BASE/'ring_thumb_release_chain_replay/research_goals.npz',allow_pickle=False)
    assert np.array_equal(z['goals'][:426],prior['goals']) and np.max(abs(np.diff(np.vstack([z['start'],z['goals']]),axis=0)))<=2+1e-9
    checks.append('503-goal extension exactly preserves the426 prefix and five full-state seams, maintains palm-inclusive support and obtains joint room without relaxing native limits')
    for geometry_name,arrival_name in (('ring_thumb_MF_third_support_geometry','ring_thumb_MF_third_support_arrival'),('ring_thumb_regrasp8_MF_geometry','ring_thumb_regrasp8_MF_arrival')):
        gp=BASE/geometry_name/'review.json';ap=BASE/arrival_name/'review.json';geometry=read(gp);arrival=read(ap)
        assert len(geometry['rows'])==3 and all(r['static_contact_candidate'] for r in geometry['rows'])
        assert geometry['object_fixed_only_in_geometry_proposals'] and geometry['finite_search_not_general_infeasibility_proof']
        assert sha(OUT/geometry['source'])==geometry['source_sha256']==arrival['source_sha256']
        assert sha(gp)==arrival['geometry_sha256'] and arrival['only_MF_commands_changed'] and arrival['cube_free_during_arrival']
        assert arrival['original_native_bounds_and_guards_preserved'] and not arrival['hardware_execution_allowed'] and arrival['final_upper_space_and_next90_not_certified']
        for r in arrival['rows']:
            if 'path' not in r:continue
            z=np.load(OUT/r['path']/'research_goals.npz',allow_pickle=False)
            assert len(z['goals'])==r['steps'] and np.max(abs(np.diff(np.vstack([z['start'],z['goals']]),axis=0)))<=2+1e-9
            assert np.max(abs(z['goals'][:,:4]-z['start'][:4]))==0. and np.max(abs(z['goals'][:,12:]-z['start'][12:]))==0.
            if r['temporary_three_finger_hold_passed']:
                assert not r['stop'] and r['samples']==4050 and min(r['finger_loaded_fractions'].values())>.8 and r['cube_shift_mm']<2 and r['orientation_change_deg']<3
    checks.append('Static MF lower-contact proposals are separated from free-object arrival trials, RF/TH commands stay fixed and final upper/turn certification remains false')
    feedback_passes=0
    for name in ('ring_thumb_MF_support_feedback','ring_thumb_MF_support_feedback_v2'):
        feedback=read(BASE/name/'review.json')
        assert feedback['only_MF_commands_changed'] and feedback['exact_model_distances_and_forces_assumed'] and feedback['not_a_hardware_sensor_contract']
        assert feedback['original_native_bounds_and_guards_preserved'] and not feedback['hardware_execution_allowed'] and feedback['new_SAC_training_steps']==0
        assert sha(OUT/feedback['source'])==feedback['source_sha256']
        for r in feedback['rows']:
            z=np.load(OUT/r['path']/'research_goals.npz',allow_pickle=False)
            assert len(z['goals'])==r['steps'] and np.max(abs(np.diff(np.vstack([z['start'],z['goals']]),axis=0)))<=2+1e-9
            assert np.max(abs(z['goals'][:,:4]-z['start'][:4]))==0. and np.max(abs(z['goals'][:,12:]-z['start'][12:]))==0.
            if r['temporary_three_finger_hold_passed']:
                feedback_passes+=1
                assert not r['stop'] and r['window_samples']==4050 and min(r['window_support_and_MF_RF_TH_fractions'])>.8 and r['window_maximum_shift_mm']<2 and r['window_maximum_angle_deg']<3
    assert feedback_passes==1
    old=read(BASE/'ring_thumb_MF_support_feedback/review.json')
    assert 'external_cube_support' in old['rows'][0]['stop'] and not old['rows'][0]['temporary_three_finger_hold_passed']
    checks.append('MF model-force feedback has one4050-sample temporary support pass; original external-support and low-load failures remain, and privileged signals are not labeled hardware sensing')
    retreat=read(BASE/'ring_thumb_three_contact_retreat8/review.json');coordinated=read(BASE/'ring_thumb_retreat_with_MF_feedback/review.json')
    assert retreat['required_contact_fingers']==['mf','th'] and len(retreat['trials'])==3
    assert not any(r['RF_clear_supported_waypoint_passed'] for r in retreat['trials'])
    assert all(r['required_finger_loaded_fractions']['mf']==0 for r in retreat['trials'][:2])
    assert all(r['whole_hand_support_fraction']==1 for r in retreat['trials'][:2])
    assert coordinated['RF_TH_command_subsequence_preserved'] and coordinated['IF_commands_bitwise_unchanged'] and coordinated['actual_state_resets_after_start']==0
    assert coordinated['original_native_bounds_and_guards_preserved'] and coordinated['privileged_model_distances_and_forces_assumed'] and not coordinated['hardware_execution_allowed']
    assert sha(OUT/coordinated['source'])==coordinated['source_sha256'] and sha(OUT/coordinated['retreat_goals'])==coordinated['retreat_goals_sha256']
    assert sha(BASE/'ring_thumb_retreat_with_MF_feedback/source_servo_MF_temporary_support.py')==coordinated['feedback_helper_sha256']
    original=np.load(OUT/coordinated['retreat_goals'],allow_pickle=False)['goals']
    for r in coordinated['rows']:
        assert not r['coordinated_three_finger_retreat_passed'] and 'self_mm' in r['stop']
        assert any('cadtip_mf_th' in name for name in r['terminal_guard']['self_pairs_mm'])
        z=np.load(OUT/r['path']/'research_goals.npz',allow_pickle=False);indices=np.r_[0:4,12:16]
        expected=original[np.minimum(np.arange(len(z['goals'])),len(original)-1)]
        assert np.array_equal(z['goals'][:,indices],expected[:,indices])
        assert np.max(abs(np.diff(np.vstack([z['start'],z['goals']]),axis=0)))<=2+1e-9
        assert np.max(abs(z['goals'][:,4:8]-z['start'][4:8]))==0.
    checks.append('Three frozen-MF retreat failures and two coupled-feedback MF/TH proxy failures remain; full-hand support alone is not mislabeled as preserved MF load')
    published=read(BASE/'ring_front_thumb_progress_20261111/review.json')
    for name,digest in published['source_reports_sha256'].items():assert sha(OUT/name)==digest,(name,'Report changed after publication')
    assert not published['first_to_next_same_station_cycle_passed'] and not published['hardware_execution_allowed']
    checks.append('Latest published report hashes match the underlying results and keep full-cycle/hardware completion false')
    report=dict(status='passed',checks=checks,verification_is_saved_evidence_audit_not_new_dynamics=True,full_two_turn_cycle_passed=False,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False)
    (BASE/'ring_front_thumb_progress_20261111/verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');return report


if __name__=='__main__':print(json.dumps(verify(),ensure_ascii=False),flush=True)
