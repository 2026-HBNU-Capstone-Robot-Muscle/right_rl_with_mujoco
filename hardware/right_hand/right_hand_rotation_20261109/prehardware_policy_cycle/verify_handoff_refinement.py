"""Audit convergence and continuous-chain evidence; never a motion approval."""
from pathlib import Path
import json
import hashlib
import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
F=OUT/'next_cycle/handoff_refinement_20261110'


def load(path):return json.loads(path.read_text(encoding='utf-8'))
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    checks=[]
    variant=load(F/'collision_convergence/manifest.json')
    assert variant['changed_options']=={'ccd_iterations':[10,50]}
    assert variant['all_model_arrays_unchanged'] and len(variant['unchanged_model_array_fields'])==468
    assert sha(ROOT/variant['source_provenance']['model_path'])==variant['source_model_sha256']
    assert sha(OUT/variant['variant_model_path'])==variant['variant_model_sha256']
    assert variant['source_file_preserved'] and not variant['hardware_execution_allowed']
    checks.append('Separate model changes only CCD iteration cap10 to50;468 arrays and original frozen model hash retained')
    ccd=load(F/'CCD_comparison.json')
    original=next(r for r in ccd['cases'] if r['variant']=='original' and 'search0/' in r['source'])
    for name in ('if_tip','rf_tip'):
        d=next(d for d in original['distances'] if d['geom']==name)
        assert d['native_distance_mm']<0 and not d['contacts']
    assert original['native_ccd_enabled'] and original['ccd_iterations']==10 and ccd['native_ccd_disable_bit']==131072
    high=[r for r in ccd['cases'] if r['ccd_iterations'] in (50,100,200) and 'search0/' in r['source']]
    assert {r['ccd_iterations'] for r in high}=={50,100,200}
    for r in high:
        for name in ('if_tip','rf_tip'):assert next(d for d in r['distances'] if d['geom']==name)['contacts']
    checks.append('Same shallow-overlap pose lacks IF/RF contacts at10 and includes them at50/100/200; original native CCD was enabled')
    replay50=load(F/'ccd_replay50/review.json');replay100=load(F/'ccd_replay100/review.json')
    assert replay50['goals_completed']==replay100['goals_completed']==4118
    assert replay50['source_goals_sha256']==replay100['source_goals_sha256']
    assert max(replay100['end_difference_from50'].values())==0
    state50=np.load(F/'ccd_replay50/end_state.npz',allow_pickle=False)
    state100=np.load(F/'ccd_replay100/end_state.npz',allow_pickle=False)
    for key in ('qpos','qvel','ctrl','act','qacc_warmstart'):
        assert np.array_equal(state50[key],state100[key])
    fresh=load(OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_active_ccd50/review.json')
    assert fresh['steps']==4118 and not fresh['first_rotation_and_hold_passed']
    assert fresh['no_state_reset_after_initial_placement'] and 'cube_translation_over80mm' in fresh['stop']
    assert not fresh['complete_cycle_passed'] and not fresh['hardware_execution_allowed']
    checks.append('Same4118 commands converge identically at50/100; independent live-policy first-turn failure remains explicit')
    chain=load(F/'connected_chain_through_v6/review.json')
    assert chain['goals']==9049 and chain['prefix_live_controller_goals']==6956 and chain['prefix_goal_mismatches']==0
    assert chain['actual_model_state_not_reset_after_initial_placement'] and chain['continuous_dynamics_reproduction_passed']
    assert all(max(s['max_abs'].values())==0 for s in chain['seams'])
    assert chain['first_to_handoff_target90_error_deg']>5 and chain['rf_th_base_support_fraction']<.8
    assert chain['suffix_is_not_a_learned_closed_loop_policy'] and not chain['whole_cycle_ready_for_hardware']
    checks.append('9049-goal initial-placement chain has zero seam errors but fails task pose/load; stored suffix is not SAC')
    dwell=load(F/'dwell_cutoff_search/review.json')
    assert len(dwell['comparisons'])==48 and all(not r['load_dwell_passed'] for r in dwell['comparisons'])
    assert sha(OUT/dwell['source_state'])==dwell['source_sha256']
    assert sha(OUT/dwell['source_goals'])==dwell['source_goals_sha256']
    endpoint=load(F/'endpoint_screen/review.json')
    assert len(endpoint['screen_trials'])==81 and sum(r['passed'] for r in endpoint['qualified_endpoints'])==3
    checks.append('Three initialized endpoint holds are separate from arrival paths;48 transient-load dwell comparisons all failed')
    summary=load(F/'summary.json')
    assert not summary['whole_cycle_ready_for_hardware'] and not summary['third_support_motion_passed']
    for r in summary['feedback_branches']:
        assert not r['passed'] and sha(OUT/r['path'])==r['sha256']
    training=OUT/'sac_ccd50_repair_10k'
    if (training/'result.json').exists():
        r=load(training/'result.json');p=load(training/'progress.json')
        assert r['steps']==10000 and p['steps']>=r['steps'] and not r['task_success_certified']
        assert r['numerical_CCD_iterations']==50 and r['integer_encoder_observations']
        assert r['max_goal_ticks']==2 and r['control_seconds']==.075 and r['devices_opened']==0
        assert all(c['packed_roundtrip_max_abs']==0 and c['one_step_goal_delta_ticks']<=2 for c in r['contract_checks'])
        for name in ('ccd_repair_env.py','train_ccd_repair.py'):
            assert sha(OUT/name)==sha(training/('source_'+name))
        checks.append('Completed SAC repair budget10000 is separate from trainer counter and learning-window success; exact state and2tick contract checked')
    parent_path=OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_active_ccd50/research_trace.npz'
    parent_goals=np.load(parent_path,allow_pickle=False)['goals']
    limits=np.load(OUT/'next_cycle/staged_shadow_bundle_v2_ccd50/reference.npz',allow_pickle=False)
    trials=[]
    for path in sorted(OUT.glob('next_cycle/staged_model_*ccd50repair*/review.json')):
        r=load(path);goals=np.load(path.parent/'research_trace.npz',allow_pickle=False)['goals']
        prefix=int(r['rotation_repair_phase_gate'][0])
        assert np.array_equal(goals[:prefix],parent_goals[:prefix])
        assert np.max(abs(np.diff(goals,axis=0)))<=2
        assert np.all(goals>=limits['lower']) and np.all(goals<=limits['upper'])
        actor=OUT/r['rotation_repair_actor_path'];manifest=load(actor.parent/'manifest.json')
        assert sha(actor)==r['rotation_repair_actor_sha256']==manifest['actor_sha256']
        assert manifest['parity_passed'] and manifest['max_action_difference']<1e-5
        assert r['model_kind']=='active_ccd50' and r['integer_encoder_observations']
        assert r['no_state_reset_after_initial_placement'] and not r['hardware_execution_allowed']
        assert not r['complete_cycle_passed']
        common=min(len(goals),len(parent_goals))
        changes=goals[prefix:common]-parent_goals[prefix:common]
        trials.append(dict(path=path.relative_to(OUT).as_posix(),steps=r['steps'],
                          passed=r['first_rotation_and_hold_passed'],stop=r['stop'],
                          changed_goal_rows_after_gate_in_common_prefix=int(np.count_nonzero(np.any(changes!=0,axis=1))),
                          max_goal_difference_from_parent_ticks=int(np.max(abs(changes))) if changes.size else 0))
    if trials:checks.append('Repair exports have NumPy/JAX parity; fresh trials preserve their declared unmodified prefix and all native goal/rate limits')
    long=OUT/'sac_ccd50_recovery_long_10k'
    if (long/'result.json').exists():
        r=load(long/'result.json');p=load(long/'progress.json')
        assert r['steps']==10000 and p['steps']>=10000 and r['horizon']==1200
        assert r['all_training_windows_cross_parent_failure4118']
        assert all(x['end']>4118 for x in r['tests']) and {x['first'] for x in r['tests']}=={3000,3400,3600}
        assert r['valid_step_reward_lower_bound']==.08 and r['fault_reward']==-20
        assert r['max_goal_ticks']==2 and r['control_seconds']==.075 and not r['task_success_certified']
        for name in ('ccd_recovery_env.py','train_ccd_recovery.py'):
            assert sha(OUT/name)==sha(long/('source_'+name))
        checks.append('Long recovery windows all extend beyond4118; valid-step reward remains positive without relaxing native guards')
    early_pool=load(F/'early_CCD50_states/review.json')
    assert early_pool['sampled_states']==37 and early_pool['goals']==3600
    assert max(early_pool['source3600_end_max_abs'].values())==0
    assert sha(F/'early_CCD50_states/state_pool.npz')==early_pool['state_pool_sha256']
    early=OUT/'sac_ccd50_early_recovery_10k'
    if (early/'result.json').exists():
        r=load(early/'result.json');p=load(early/'progress.json')
        assert r['steps']==10000 and p['steps']>=10000 and r['horizon']==1200
        assert r['reset_indices']==[600,900,1200,1600,2000,2600,3000,3400]
        assert {x['first'] for x in r['tests']}==set(r['reset_indices'])
        assert r['valid_step_reward_lower_bound']==.08 and r['max_goal_ticks']==2
        assert not r['task_success_certified'] and r['devices_opened']==0
        for name in ('ccd_early_recovery_env.py','train_ccd_early_recovery.py'):
            assert sha(OUT/name)==sha(early/('source_'+name))
        checks.append('Early repair uses37 exactly reproduced CCD50 states, all8 reset windows checked and a separately recorded10000 SAC budget')
    authority=load(F/'action_authority_audit.json')
    assert abs(authority['residual_ticks_before_rounding']-.01/(2*np.pi/4096))<1e-12
    assert authority['executed_per_cycle_limit_ticks']==2 and authority['cycle_seconds']==.075
    assert authority['initialized_endpoint_is_one_candidate_not_a_minimum_required_motion']
    assert authority['does_not_prove_all_existing_policies_useless_or_new_training_mandatory']
    sampling=load(F/'observation_timing_audit.json')
    assert len(sampling['rows'])==3 and sampling['all_compared_next_goals_match']
    assert all(a['different_next_joint_goals']==0 for r in sampling['rows'] for a in r['actors'])
    checks.append('Proposal authority differs from2tick execution rate; limited pose-cache probes match joint goals without claiming full sensor equivalence')
    small_path=F/'small_joint_response/review.json'
    if small_path.exists():
        small=load(small_path);answer=load(F/'small_joint_response/answer_summary.json')
        assert small['total_comparisons']==small['valid_comparisons']==96 and len(small['samples'])==3
        assert small['max_goal_increment_ticks']==2 and small['control_seconds']==.075 and small['total_control_seconds']==1.275
        assert sha(small_path)==answer['probe_report_sha256']
        for sample in small['samples']:
            assert not sample['baseline']['stop'] and len(sample['results'])==32
            assert {(r['motor_id'],r['offset_ticks']) for r in sample['results']}=={(mid,delta) for mid in range(1,17) for delta in (-7,7)}
            assert all(not r['stop'] and r['comparison_valid'] and r['completed_goals']==17 for r in sample['results'])
            assert max(r['cube_angle_change_vs_matched_hold_deg'] for r in sample['results'])==sample['maximum_cube_angle_change_deg']
        assert answer['small_residual_radius_not_established_as_primary_failure_cause']
        assert answer['recorded_ring_ticks']==[3018,2961,2829,3053]
        assert sha(ROOT/'ring_reference_readonly_20261109_124719_945552/result.json')==answer['ring_original_sha256']
        checks.append('96 initialized one-joint probes use matched17-cycle holds and unchanged guards; recorded RF source hash retained without a physical-error claim')
    result=dict(status='passed',checks=checks,devices_opened=0,
                fresh_repair_trials=trials,
                whole_cycle_or_hardware_success=False,original_source_model_preserved=True)
    (F/'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
