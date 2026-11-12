"""Cross-check scope, source bindings and commands in the new work evidence."""
from pathlib import Path
import hashlib
import json
import numpy as np

OUT=Path(__file__).resolve().parent;BASE=OUT/'next_cycle/handoff_refinement_20261110'
def read(path):return json.loads(path.read_text(encoding='utf-8'))
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def verify():
    checks=[];geometry=BASE/'opposing_workspace_Z_seed122/review.json';g=read(geometry)
    assert sha(OUT/g['source'])==g['source_sha256'] and sha(OUT/g['proposal'])==g['proposal_sha256']
    assert g['static_opposed_contact_candidate'] and g['translation_is_hypothetical_IK_not_executed']
    assert not g['arrival_path_passed'] and not g['free_object_hold_passed'] and not g['hardware_execution_allowed']
    assert 15<np.linalg.norm(g['cube_shift_mm'])<30
    checks.append('Alternative contact geometry keeps an explicit hypothetical translation and never claims an arrival')
    replay=read(BASE/'opposing_grip_review_20261110/hold_replay_verification.json');assert replay['status']=='passed' and len(replay['rows'])==2
    for row in replay['rows']:
        assert sha(OUT/row['declared_review'])==row['declared_review_sha256']
        assert max(row['final_bound_state_errors'].values())==0
        assert row['hold_samples']==row['upper_certified_samples']==row['whole_hand_supported_samples']==row['MF_loaded_samples']==row['TH_loaded_samples']==4050
        assert row['minimum_upper_lower_mm']>.3 and row['station_drift_mm']<2 and row['orientation_drift_deg']<3
        assert row['arrival_path_not_evaluated']
    checks.append('Independent90/180 initialized hold replays reproduce every bound state exactly and all4050 support/load/upper-space samples')
    load=read(BASE/'opposing_grip_load_hypotheses/review.json');assert load['alternative_source_explicit'] and load['station_scope']=='source'
    assert load['applied_moment_is_hypothesis_not_measured_left_hand_load'] and load['model_torque_is_not_calibrated_to_450mA']
    assert len(load['cases'])==11 and sum(r['conditional_hold_passed'] for r in load['cases'])==10
    assert not next(r for r in load['cases'] if r['assumed_constant_moment_about_up_Nm']==.01)['conditional_hold_passed']
    for r in load['cases']:
        assert r['original_guards_and_joint_limits_preserved'] and r['only_sliding_friction_entry_changed']
        if r['conditional_hold_passed']:assert r['samples']==r['certified_window_samples']==4050 and r['upper_minimum_lower_mm']>.3
    checks.append('Ten of eleven separately initialized load/friction hypotheses pass; positive0.01Nm failure and physical-force uncertainty remain explicit')
    arrival=read(BASE/'opposing_workspace_Z_arrivals/review.json');assert len(arrival['rows'])==4
    assert arrival['cube_free_in_every_actual_timestep'] and not arrival['initial_placement_first_rotation_and_two_turn_cycle_passed']
    for row in arrival['rows']:
        assert not row['alternative_station_arrival_and_hold_passed'] and row['alternative_station_error_mm']>15
        z=np.load(OUT/row['path']/'research_goals.npz',allow_pickle=False)
        assert len(z['goals'])==row['steps'] and np.max(abs(np.diff(np.vstack([z['start'],z['goals']]),axis=0)))<=2+1e-9
    checks.append('Four true free-object rate-limited arrivals retain their station failures instead of being relabeled by cold holds')
    actual=read(BASE/'actual_reached_workstation_hold_audit/review.json');assert len(actual['rows'])==3
    for r in actual['rows']:
        assert sha(OUT/r['source'])==r['source_sha256']
        assert r['support_space_pose_hold_at_reached_station_passed'] and r['hold_samples']==r['upper_certified_samples']==4050
        assert r['canonical_station_error_mm']>2 and r['old_canonical_task_not_reclassified']
    checks.append('Reached90/180 states pass support and upper-space at their own positions; old2mm canonical metric failures remain')
    reached_load=read(BASE/'reached_workstation_load_hypotheses/review.json')
    assert reached_load['station_scope']=='source' and reached_load['alternative_source_explicit']
    assert len(reached_load['cases'])==11 and sum(r['conditional_hold_passed'] for r in reached_load['cases'])==9
    assert sha(OUT/reached_load['source'])==reached_load['source_sha256']
    assert all(r['conditional_hold_passed']==(abs(r['assumed_constant_moment_about_up_Nm'])<.01) for r in reached_load['cases'])
    checks.append('The actually reached first grip has its own9/11 load/friction result; its two0.01Nm failures are separate from the initialized opposed grip')
    firstpath=BASE/'initial_to_reached_workstation90/review.json'
    if firstpath.exists():
        first=read(firstpath);assert first['steps']==3015 and first['first_rotation_and_reached_station_hold_passed']
        assert first['actual_state_resets_after_initial_placement']==0 and len(first['seams'])==4
        assert all(max(s['bound_state_errors'].values())==0 for s in first['seams'])
        assert first['hold_samples']==first['certified_upper_samples']==4050 and first['minimum_upper_lower_mm']>.3
        assert 5<first['canonical_station_error_mm']<6 and not first['original_canonical_station_2mm_task_passed']
        assert not first['second_rotation_passed'] and not first['full_two_turn_cycle_passed'] and not first['hardware_execution_allowed']
        for name,digest in first['source_files_sha256'].items():assert sha(OUT/name)==digest
        z=np.load(firstpath.parent/'research_goals.npz',allow_pickle=False)
        assert len(z['goals'])==3015 and np.max(abs(np.diff(np.vstack([z['start'],z['goals']]),axis=0)))<=2+1e-9
        assert first['prefix_saved_policy_goals']==1700 and first['model_teacher_goals']==1261 and first['position_hold_goals']==54
        checks.append('Initial placement first90 replay has3015 commands and four exact seams, with an explicit reached workstation and separate policy/teacher/hold parts')
    next_path=BASE/'reached_workstation_next90/review.json'
    if next_path.exists():
        r=read(next_path);assert r['steps']==234 and r['stop']=='horizon_objective_stalled' and not r['conditional_bridge_and_hold_passed']
        assert r['final']['error_deg']>80 and r['actual_state_resets_after_start']==0
        assert r['alternative_goal_reference_explicit'] and r['center_target_scope']=='reference'
        assert sha(OUT/r['source'])==r['source_sha256'] and sha(OUT/r['goal_reference'])==r['goal_reference_sha256']
        goals=np.load(next_path.parent/'research_goals.npz',allow_pickle=False)
        assert np.max(abs(np.diff(np.vstack([goals['start'],goals['goals']]),axis=0)))<=2+1e-9
        checks.append('Next rotation from the actual first hold retains its234-goal stall and fixed-before-run station; no full-cycle success is inferred')
    report=dict(status='passed',checks=checks,full_two_turn_cycle_passed=False,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False)
    (BASE/'opposing_grip_review_20261110/verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    return report


if __name__=='__main__':print(json.dumps(verify(),ensure_ascii=False),flush=True)
