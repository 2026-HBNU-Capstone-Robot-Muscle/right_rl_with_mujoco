"""Audit robustness experiments and no-output integration without opening devices."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
import ast
import hashlib
import json
import numpy as np

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1];BASE=OUT/'next_cycle/handoff_refinement_20261110';FOLDER=BASE/'robust_cycle_preparation_20261111'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_text(encoding='utf-8'))


def verify():
    checks=[];sources={}
    def report(path):
        r=read(path);sources[path.relative_to(OUT).as_posix()]=sha(path)
        assert r.get('devices_opened',0)==0 and r.get('new_SAC_training_steps',0)==0 and not r.get('hardware_execution_allowed',False)
        if 'source' in r and 'source_sha256' in r:assert sha(OUT/r['source'])==r['source_sha256']
        return r
    bounds=np.load(FOLDER/'command_contract.npz',allow_pickle=False);low=bounds['lower'];high=bounds['upper']
    def commands(path,length=None):
        z=np.load(path,allow_pickle=False);values=np.vstack([z['start'],z['goals']]);assert values.shape[1]==16 and np.isfinite(values).all()
        assert np.max(abs(np.diff(values,axis=0)))<=2+1e-9 and np.all(values>=low) and np.all(values<=high),path
        if length is not None:assert len(z['goals'])==length
    capture=report(FOLDER/'failure_checkpoints/review.json');assert len(capture['rows'])==3
    for r in capture['rows']:
        p=OUT/r['path'];pool=np.load(p/'state_pool.npz',allow_pickle=False);assert sha(p/'state_pool.npz')==r['state_pool_sha256']
        assert 1700 in pool['indices'] and not any(pool['guard_gap'].ravel()>.25)
        if r['case']=='initial_cube_worldX_plus1mm':assert r['goals']==1835 and r['stop']==['self_mm'] and r['exact_prior_failure_reproduced']
        if r['case']=='sliding_mu0p25':assert r['goals']==2272 and r['stop']==['cube_translation_over80mm'] and r['exact_prior_failure_reproduced']
    contact=report(FOLDER/'contact_diagnosis/review.json');assert [r['CCD_iterations'] for r in contact['rows']]==[50,100,200]
    assert contact['rows'][0]['exact_failure_qpos_match']
    depths=[r['self_pairs_mm']['mf_bs_collision_1|mf_tip'] for r in contact['rows']]
    assert max(depths)-min(depths)==0 and 0<depths[0]<.001
    assert all(abs(r['static_before_rejected_step']['native_geom_distance_mm']-r['static_before_rejected_step']['independent_convex_axis_gap_mm'])<1e-8 for r in contact['rows'])
    checks.append('Perturbed failures exactly reproduce at1835/2272 goals; the same1ms MF overlap is unchanged at CCD50/100/200 and agrees with independent convex geometry')
    fk=report(FOLDER/'encoder_clearance/review.json');assert fk['fixture_states']==140 and fk['maximum_position_difference_m']<1e-12 and fk['maximum_gap_difference_mm']<1e-8
    assert sha(OUT/fk['kinematics'])==fk['kinematics_sha256']
    def core(path):
        tree=ast.parse(path.read_text(encoding='utf-8'));cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='FingerClearance')
        return {n.name:ast.dump(n,include_attributes=False) for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('poses','gap_mm')}
    assert core(OUT/'cycle_joint_clearance.py')==core(FOLDER/'encoder_clearance/source_cycle_joint_clearance.py')
    checks.append('140-state native geometry parity retains exactly the same FK/gap core after adding stricter input validation')
    adapter=report(FOLDER/'observation_adapter_verification.json');assert adapter['status']=='passed' and sha(OUT/'cycle_observation_adapter.py')==adapter['source_sha256'] and not adapter['physical_registration_verified']
    gates=report(FOLDER/'runtime_gate_verification.json');assert gates['status']=='passed' and len(gates['tests'])==13 and gates['motor_commands_sent']==0
    parity=report(FOLDER/'encoder_pose_feedback_v1_complete_bundle/shadow_fixture_verification.json');assert parity['status']=='passed' and parity['goal_comparisons']==15534
    for r in parity['cases']:assert r['exact_mismatches']==0 and sha(OUT/r['fixture'])==r['fixture_sha256']
    checks.append('Observer transforms, freshness and legacy checked-state bridge pass synthetic tests;13 runtime gates and15534 exact Windows goals retain no-output and unverified-camera boundaries')
    bundles=('shadow_bundle','shadow_bundle_pose_v2','shadow_bundle_tip_trajectory_v3')
    for name in bundles:
        folder=FOLDER/name;meta=read(folder/'bundle.json');assert meta['accepted_registration_id'] is None and not meta['hardware_execution_allowed']
        assert meta['max_goal_ticks']==2 and meta['control_seconds']==.075 and meta['current_limit_mA']==450 and not meta['PID_changed']
        assert meta['final_hold_keeps_command'] and not meta['automatic_open'] and not meta['automatic_torque_off']
        for n,digest in meta['files_sha256'].items():assert sha(folder/n)==digest,(name,n)
    checks.append('Three immutable research bundles preserve original SAC files,450mA metadata,2tick/75ms limits, PID and terminal hold; all reject unverified real registration')
    from publish_robust_cycle_preparation import BATCHES,TEACHERS
    batches=0;passes=0;failures=0
    for name in BATCHES:
        path=FOLDER/name/'review.json'
        if not path.exists():continue
        batch=report(path)
        for n,digest in batch.get('source_files_sha256',{}).items():assert sha(OUT/n)==digest
        if 'bundle' in batch:assert sha(OUT/batch['bundle']/'bundle.json')==batch['bundle_metadata_sha256']
        for r in batch['rows']:
            commands(OUT/r['path']/'research_goals.npz',r['steps']);batches+=1;passed=bool(r.get('two_turn_reached_location_holds_passed',False));passes+=passed;failures+=not passed
            if passed:
                if 'hold_samples' in r:
                    assert r['hold_samples']==r['hold_upper_certified_samples']==r['hold_supported_samples']==[4050,4050]
                    assert r['first90_error_deg']<5 and r['final180_error_deg']<5
                assert not r['stop']
            if r.get('proposal_latency_ms'):assert r['proposal_latency_ms']['not_a_realtime_certificate']
    assert batches>=14 and passes>=4 and failures>=10
    checks.append('All actual finite controller comparisons retain their native outcomes and command bounds; nominal success is not substituted for unresolved1mm/friction failures')
    live=read(FOLDER/'live_SAC_prefix/review.json');nominal=next(r for r in live['rows'] if r['case']=='nominal');assert nominal['nominal_prefix_exact_match'] and nominal['changed_prefix_goal_rows']==0
    assert all(r['changed_prefix_goal_rows']>0 and not r['two_turn_reached_location_holds_passed'] for r in live['rows'] if r['case']!='nominal')
    first=report(FOLDER/'low_friction_first_hold_state_v4/review.json')
    assert sha(OUT/first['source'])==first['source_sha256']
    assert first['replayed_goal_count']==2478 and first['first_hold_goal_count']==54
    assert first['trajectory_qpos_difference']==0 and first['first90_error_deg']<5
    assert first['actual_state_resets_after_initial_placement']==0
    checkpoint=np.load(FOLDER/'low_friction_first_hold_state_v4/end_state.npz',allow_pickle=False)
    original=np.load(FOLDER/'low_friction_teacher_cycle_connection_v4/keep_teacher_pose/trajectory.npz',allow_pickle=False)['qpos']
    assert np.array_equal(checkpoint['qpos'],original[2478])
    checks.append('The low-friction first-hold checkpoint exactly matches the 2478-goal full-placement replay and retains the verified4050-sample first hold')
    load=report(FOLDER/'low_friction_next_turn_contact_diagnosis_v4/review.json')
    assert load['status']=='restored_model_contact_diagnosis_not_physical_grip_certificate'
    assert [r['stage'] for r in load['rows']]==['first90_hold','failed_second180_hold']
    for row in load['rows']:
        assert sha(OUT/row['source'])==row['source_sha256']
        assert set(row['normal_load_by_prefix_N'])=={'if'}
        assert row['supported_sample']
    checks.append('Restored low-friction states support the cube at the index base only; support is not counted as a loaded thumb/middle/ring rotation grip')
    fit=report(FOLDER/'local_pose_response/review.json');assert fit['phases']==37 and fit['nominal_branch_maximum_qpos_difference']==0 and fit['finite_difference_fit_is_not_SAC_or_distillation']
    assert sha(OUT/fit['response'])==fit['response_sha256']
    checks.append('Live110D SAC prefix parity and failed changed-input tests are explicit;37 local finite-difference fits are neither new SAC nor distillation')
    for name in TEACHERS:
        path=BASE/name/'review.json'
        if not path.exists():continue
        r=report(path);assert r['sliding_friction_hypothesis']==.25 and r['only_sliding_friction_entry_changed'] and r['middle_self_planning_pair_included']
        assert r['original_native_guards_and_joint_limits_preserved'] and r['actual_state_resets_after_start']==0
        commands(path.parent/'research_goals.npz',r['steps'])
        assert r['worst']['self_mm']<=1e-6 and r['worst']['tracking_rad']<=.18 and r['worst']['range_rad']<=1e-6
    old=report(FOLDER/'low_friction_loaded_waypoint_hold_v1/review.json')
    assert sha(OUT/old['parent_goals'])==old['parent_goals_sha256']
    assert old['waypoint_step']==368 and old['source_trajectory_qpos_difference']==0
    assert old['hold_samples']==1394 and old['stop']==['range_rad'] and not old['waypoint_loaded_hold_passed']
    diag=report(FOLDER/'low_friction_waypoint_failure_diagnosis_v1/review.json')
    assert diag['rows'][1]['motor_id']==6 and 1e-6<diag['rows'][1]['largest_signed_range_excess_rad']<1.1e-6
    assert diag['upper_space_is_diagnostic_at_intermediate_waypoint_not_required_during_rotation']
    reserve=report(FOLDER/'low_friction_waypoint_joint_reserve_v1/review.json')
    assert sha(OUT/reserve['parent_goals'])==reserve['parent_goals_sha256']
    assert reserve['original_native_guards_and_joint_limits_preserved'] and reserve['actual_state_resets_after_first_hold']==0
    assert [r['ID6_reserve_ticks'] for r in reserve['rows']]==[0,1,2]
    for r in reserve['rows']:
        commands(OUT/r['path']/'hold_goals.npz')
        if r['ID6_reserve_ticks']==0:assert r['hold_samples']==1394 and r['stop']==['range_rad']
        else:
            assert r['loaded_MF_TH_hold_passed'] and r['stable_support_hold_passed'] and not r['stop']
            assert r['hold_samples']==r['supported_samples']==4050
            assert r['MF_loaded_samples']/4050>.8 and r['TH_loaded_samples']/4050>.8
            assert r['maximum_shift_mm']<2 and r['maximum_drift_deg']<3
            assert r['worst']['range_rad']<=1e-6
    checks.append('Unchanged ID6 range guard is respected by a1tick command reserve; two4050-sample loaded intermediate holds pass without requiring upper sweep clearance during rotation')
    for name in ('robust_mu025_next_turn_teacher_v1','robust_mu025_next_turn_teacher_v2','robust_mu025_next_turn_teacher_v3'):
        path=BASE/name/'review.json'
        if not path.exists():continue
        r=report(path)
        assert r['mode']=='next' and r['global_yaw_target_deg']==180 and r['sliding_friction_hypothesis']==.25
        assert r['original_native_guards_and_joint_limits_preserved'] and r['actual_state_resets_after_start']==0
        assert r['full_initial_placement_two_turn_cycle_passed'] is False
        commands(path.parent/'research_goals.npz',r['steps'])
        assert r['worst']['self_mm']<=1e-6 and r['worst']['tracking_rad']<=.18 and r['worst']['range_rad']<=1e-6
        if name.endswith('_v3'):
            assert r['ID6_upper_command_reserve_ticks']==1 and r['command_reserve_does_not_change_native_joint_or_guard_limits']
            assert np.max(np.load(path.parent/'research_goals.npz',allow_pickle=False)['goals'][:,5])<=np.floor(high[5])-1
    checks.append('Separate low-friction next-turn teachers retain source-state hashes, native guards and2tick bounds without being counted as a continuous full cycle')
    published=report(FOLDER/'review.json')
    for n,digest in published['source_reports_sha256'].items():assert sha(OUT/n)==digest,(n,'Changed after publication')
    assert published['user_deferred_7p4mm_position_shift_to_left_hand_integration']
    assert published['original2mm_miss_preserved_but_not_added_as_standalone_prerequisite']
    assert published['remaining_work']['physical_camera_registration']=='unverified'
    result=dict(status='passed',checks=checks,source_reports_sha256=sources,native_controller_case_rows=batches,nominal_case_passes=passes,retained_nonpassing_cases=failures,Windows_goal_comparisons=15534,new_SAC_training_steps=0,verification_is_saved_evidence_and_no_output_contracts_not_physical_success=True,devices_opened=0,hardware_execution_allowed=False)
    (FOLDER/'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8');return result


if __name__=='__main__':print(json.dumps(verify(),ensure_ascii=False),flush=True)
