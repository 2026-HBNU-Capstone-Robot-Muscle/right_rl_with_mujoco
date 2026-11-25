"""Check saved research artifacts and inventory them without opening devices."""
from pathlib import Path
from html.parser import HTMLParser
from urllib.parse import unquote, urlsplit
import ast
import hashlib
import json
import numpy as np
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from evaluator_revision_evidence import validate_evaluator_revision

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


class Links(HTMLParser):
    def __init__(self):
        super().__init__()
        self.links = []

    def handle_starttag(self, tag, attrs):
        for key, value in attrs:
            if key in ('href', 'src') and value:
                self.links.append(value)


def main():
    checks = []
    for path in sorted(OUT.rglob('*.py')):
        ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    checks.append('All new and saved Python source parses without execution')
    for relative in ('review/index.html', '../policy_audit/explanation/index.html',
                     'next_cycle/handoff_refinement_20261110/index.html',
                      'next_cycle/handoff_refinement_20261110/whole_hand_preflight_v2/index.html',
                       'next_cycle/handoff_refinement_20261110/RF_clear_work_grip_evidence/index.html',
                       'next_cycle/handoff_refinement_20261110/bridge_progress_20261110/index.html',
                       'next_cycle/handoff_refinement_20261110/opposing_grip_review_20261110/index.html',
                       'next_cycle/handoff_refinement_20261110/ring_front_thumb_progress_20261111/index.html',
                       'next_cycle/handoff_refinement_20261110/first_to_next_contact_bridge_20261111/index.html',
                       'next_cycle/handoff_refinement_20261110/MF_TH_interference_diagram_20261111/index.html',
                       'next_cycle/handoff_refinement_20261110/robust_cycle_preparation_20261111/index.html',
                       'next_cycle/handoff_refinement_20261110/reposition_then_second_rotation_20261111/index.html'):
        path = (OUT / relative).resolve()
        parser = Links(); parser.feed(path.read_text(encoding='utf-8'))
        for value in parser.links:
            link = urlsplit(value)
            if link.scheme or link.netloc or not link.path:
                continue
            destination = (path.parent / unquote(link.path)).resolve()
            if not destination.exists():
                raise AssertionError((str(path), value))
    checks.append('Local report and original explanation links resolve')
    bridge_path=OUT/'next_cycle/handoff_refinement_20261110/clear_grip_next5_canonical_joined_chain/review.json'
    if bridge_path.exists():
        from verify_clear_bridge_evidence import verify
        bridge_checks=verify();assert bridge_checks['status']=='passed'
        checks.append('Additional bridge audit validates804 conditional goals, exact state seams, seven planning-axis cases and retained governed-policy/regrasp failures')
    opposed=OUT/'next_cycle/handoff_refinement_20261110/opposing_grip_review_20261110/hold_replay_verification.json'
    if opposed.exists():
        from verify_opposing_grip_results import verify as verify_opposed
        assert verify_opposed()['status']=='passed'
        checks.append('Opposed-grip initialized hold replays,11 load hypotheses, failed arrivals and reached-station diagnostics preserve scope and command limits')
    ring_front=OUT/'next_cycle/handoff_refinement_20261110/ring_front_thumb_progress_20261111/review.json'
    if ring_front.exists():
        from verify_ring_front_thumb_results import verify as verify_ring_front
        assert verify_ring_front()['status']=='passed'
        checks.append('Photo-front RF/TH holds, initialized637-goal alternative and426-goal intermediate release replay retain distinct task scopes and native command bounds')
    summary = read(OUT / 'evaluation_summary.json')
    robust=OUT/'next_cycle/handoff_refinement_20261110/robust_cycle_preparation_20261111/review.json'
    if robust.exists():
        from verify_robust_cycle_preparation import verify as verify_robust
        assert verify_robust()['status']=='passed'
        checks.append('Robustness failures reproduce exactly;140-state FK,128 synthetic coordinate fixtures,13 runtime gates and15534 Windows no-output goals retain original guards and unresolved physical registration')
    entry=OUT/'next_cycle/handoff_refinement_20261110/reposition_then_second_rotation_20261111/review.json'
    if entry.exists():
        from verify_rotation_entry_reposition import verify as verify_entry
        assert verify_entry()['status']=='passed'
        checks.append('Actual entry mismatch, position-first transport, finger alignment orders and reused second-turn comparisons retain original guards, finite outcomes and explicit initialization-control scope')
    whole_cycle=OUT/'next_cycle/handoff_refinement_20261110/first_to_next_contact_bridge_20261111/review.json'
    if whole_cycle.exists():
        from verify_whole_hand_cycle_results import verify as verify_whole_cycle
        assert verify_whole_cycle()['status']=='passed'
        checks.append('Whole-hand contact changes,4673 partial and5089 two-hold replays have exact seams and unchanged2tick commands; nominal reached-location success, common-station miss and finite perturbation failures retain distinct scopes')
    assert summary['new_training_steps']==sum(r['steps'] for r in summary['training'])
    prior=[r for r in summary['training'] if not r['path'].startswith(('sac_ccd50_repair_10k/','sac_ccd50_recovery_long_10k/','sac_ccd50_early_recovery_10k/'))]
    assert sum(r['steps'] for r in prior)==60000 and len(prior)==4
    repair=[r for r in summary['training'] if r['path'].startswith(('sac_ccd50_repair_10k/','sac_ccd50_recovery_long_10k/','sac_ccd50_early_recovery_10k/'))]
    assert len(repair)<=3 and all(r['steps']==10000 and r['numerical_CCD_iterations']==50 for r in repair)
    assert not summary['second90_rotation_passed'] and not summary['distillation_completed']
    assert summary['physical_successes'] == 0 and not summary['hardware_execution_allowed']
    checks.append('Prior four SAC budgets total60000; additional CCD50 budgets of10000 each are separate, with no next90, physical or distillation completion claim')
    connected = read(OUT / 'connected_sac_and_teacher/review.json')
    assert connected['first_rotation_and_hold_passed']
    assert connected['prefix_goal_mismatches'] == 0
    assert max(connected['prefix_seam_max_abs'].values()) == 0.
    assert connected['suffix_is_saved_model_teacher_not_SAC']
    assert not connected['next_rotation_passed'] and not connected['hardware_execution_allowed']
    assert connected['final_hold']['samples'] == 4050 and connected['final_hold']['bad'] == 0
    assert connected['final_hold']['gap'] > .3
    assert connected['final_hold']['lower']/4050 > .8
    assert connected['endpoint']['target90_error_deg'] < 5
    assert connected['common_station_error_mm'] < 2
    checks.append('Uninterrupted first-quarter evidence, exact source seam and4050 pose certificates agree')
    trace = dict(np.load(OUT / 'connected_sac_and_teacher/research_trace.npz', allow_pickle=False))
    assert len(trace['first_segment_goals']) == 6409
    assert np.max(np.abs(np.diff(trace['goals'], axis=0))) <= 2
    assert connected['reverse_joint_baseline']['stop']
    checks.append('Connected and reverse research goals retain max2tick spacing; reverse failure is retained')
    for r in summary['rows']:
        if r['case'] != 'nominal':
            assert not r['passed']
    assert {r['case'] for r in summary['rows'] if r['case']!='nominal'} == {'x_plus05','friction025','vision_x_plus1'}
    checks.append('Three tested placement/friction/vision perturbation failures are retained')
    for r in summary['small_turn_SAC_evaluations']:
        assert not r['passed'] and 5 < r['initial_error_deg'] < 6
    checks.append('Small-turn SAC failures and corrected degree units agree')
    for r in summary['model_feedback_teacher_evaluations']:
        if r['increment_deg'] == 90:
            assert not r['passed']
    checks.append('All recorded next90 model-feedback failures are retained')
    learned=summary['connected_SAC_prefix_learned_correction_and_position_hold']
    assert len(learned)==3 and {r['case'] for r in learned}=={'nominal','x_plus05','friction025'}
    for r in learned:
        assert r['first_rotation_and_hold_passed'] and r['goals_completed']==6556
        assert not r['suffix_is_saved_model_teacher_not_SAC'] and r['student_is_behavior_cloned_correction_not_SAC']
        assert r['prefix_goal_mismatches']==0 and max(r['prefix_seam_max_abs'].values())==0.
        assert r['final_hold']['samples']==4050 and r['final_hold']['bad']==0 and r['final_hold']['gap']>.3
        assert r['final_hold']['lower']/4050>.8 and r['endpoint']['target90_error_deg']<5 and r['common_station_error_mm']<2
        path=OUT/r['path'];saved=read(path)
        assert saved['student_actor_sha256']==sha(OUT/saved['student_run']/'actor.npz')
        goals=np.load(path.parent/'research_trace.npz',allow_pickle=False)['goals']
        assert len(goals)==6556 and np.max(abs(np.diff(goals,axis=0)))<=2
    checks.append('Three training-condition6556-goal connections use a learned correction, original limits and4050-state clearance evidence')
    staged=read(OUT/'next_cycle/staged_shadow_runtime_verification.json')
    bundle=OUT/'next_cycle/staged_shadow_bundle_v2';meta=read(bundle/'bundle.json')
    assert staged['status']=='passed' and staged['model_fixture_goal_comparisons']==19668
    assert staged['source_runtime_sha256']==sha(OUT/'staged_shadow_runtime.py')
    assert staged['bundle_metadata_sha256']==sha(bundle/'bundle.json')
    assert all(r['mismatched_goals']==0 for r in staged['cases'])
    assert len(staged['transaction_checks'])==27
    for name,digest in meta['files_sha256'].items():assert sha(bundle/name)==digest
    assert not meta['hardware_execution_allowed'] and not meta['complete_two_turn_cycle_validated']
    assert not meta['acquisition_metric_is_global_clearance_certificate']
    checks.append('Packaged stage sources and19668 model-fixture decisions match;27 transaction/hold checks retain no-output gating and screening is not a global margin proof')
    readiness=read(OUT/'next_cycle/readiness.json')
    assert not readiness['hardware_execution_allowed'] and not readiness['second_rotation_passed']
    packaged=summary['packaged_controller_free_model_trials']
    assert packaged and len(packaged)==len(readiness['packaged_controller_model_trials'])
    original_integer=next(r for r in packaged if r['case']=='nominal' and r['integer_encoder_observations']
                         and not r.get('correction_actor_research_override') and not r.get('native_range_admissible_correction_selection'))
    assert not original_integer['first_rotation_and_hold_passed'] and original_integer['steps']==6307
    friction=next(r for r in packaged if r['case']=='friction035')
    assert not friction['first_rotation_and_hold_passed'] and 'cube_translation_over80mm' in friction['stop']
    for r in packaged:
        assert r['exact_model_object_pose_and_geometry_assumed'] and not r['hardware_execution_allowed']
        path=OUT/r['path'];saved=read(path)
        assert sha(path)==next(x['sha256'] for x in readiness['packaged_controller_model_trials'] if x['path']==r['path'])
        assert not r['complete_cycle_passed']
        if saved.get('correction_actor_path'):assert sha(OUT/saved['correction_actor_path'])==saved['correction_actor_sha256']
    checks.append('Full packaged-controller model trials retain integer-encoder and unseen-condition failures; research overrides are explicit')
    confirmation_path=OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_schema_v2_confirmation/review.json'
    confirmed=read(confirmation_path)
    assert confirmed['first_rotation_and_hold_passed'] and confirmed['steps']==6956
    assert confirmed['bundle_metadata_sha256']==sha(bundle/'bundle.json')
    assert confirmed['integer_encoder_observations'] and confirmed['native_range_admissible_correction_selection']
    assert not confirmed['acquisition_screen_is_global_clearance_certificate']
    assert confirmed['independently_generated_goals_comparison']['mismatched_goal_rows']==0
    assert confirmed['final_hold']['samples']==4050 and confirmed['final_hold']['bad']==0 and confirmed['final_hold']['gap']>.3
    assert confirmed['final_hold']['lower']==4050 and confirmed['final']['error_deg']<5 and confirmed['final']['center_error_mm']<2
    commands=np.load(confirmation_path.parent/'research_trace.npz',allow_pickle=False)['goals']
    limits=np.load(bundle/'reference.npz',allow_pickle=False)
    assert np.all(commands>=limits['lower']) and np.all(commands<=limits['upper'])
    assert np.max(abs(np.diff(commands,axis=0)))<=2
    checks.append('Packaged admissible correction reproduced6956 live-generated goals with integer encoder readings and long hold; bounds and4050 final margin states pass')
    dataset=OUT/'next_cycle/recovery_teacher_data_long_feedback'
    data=read(dataset/'manifest.json');assert data['examples']==768 and not data['is_new_SAC_training']
    assert len(data['sources'])==3 and all(r['replay_qpos_error']==0 for r in data['sources'])
    for r in data['sources']:
        assert sha(OUT/r['source_state'])==r['source_sha256'] and sha(OUT/r['teacher_goals'])==r['teacher_sha256']
    assert all(r['training']['new_SAC_training_steps']==0 for r in summary['correction_behavior_cloning_runs_not_SAC_environment_steps'])
    checks.append('Long teacher768 sensor-contract examples reproduce exactly; behavior cloning is counted separately from SAC')
    margin=read(OUT/'next_cycle/upper_gap_metric_audit.json')
    assert margin['geometry_changes']==0 and margin['failed_geoms']
    assert any(r['inside_required_margin_upper_witness'] and r['upper_minus_lower_mm']<1e-6 for r in margin['failed_geoms'])
    checks.append('Independent nearest-pair witness confirms the unseen-case margin shortfall; slab/radial screening is not mislabeled as global clearance')
    selected = OUT / 'sac_current_contract_v4_20k/export_000000010256'
    manifest = read(selected / 'manifest.json')
    assert sha(selected / 'actor.npz') == manifest['actor_sha256']
    assert manifest['parity_passed'] and manifest['max_action_difference'] < 1e-5
    assert manifest['numpy_export_is_not_distillation'] and not manifest['hardware_execution_allowed']
    checks.append('Chosen actor checksum and NumPy/JAX parity match; export is not distillation')
    shadow = read(OUT / 'shadow_runtime_verification.json')
    # The runtime test has its own detailed contract assertions; do not replace
    # them with a superficial imitation of the implementation here.
    assert shadow.get('passed', shadow.get('status') == 'passed')
    checks.append('Existing no-device shadow runtime contract result is present')
    calibration = read(OUT / 'saved_calibration_asset_review.json')
    assert len(calibration['camera_intrinsic_candidates']) == 3
    assert all(r['finite_positive_intrinsic_candidate'] for r in calibration['camera_intrinsic_candidates'])
    assert not calibration['physical_camera_registration_verified']
    checks.append('Existing camera calibration assets inspected; unverified external registration remains explicit')
    frozen_path = ROOT / 'team_integration_20261109/middleware_review/verification.json'
    frozen = read(frozen_path)
    assert frozen['frozen_execution_bindings_verified'] == 170 and not frozen['frozen_execution_changes']
    assert frozen['cached_source_files_verified'] == 57 and not frozen['cached_source_changes']
    assert not frozen['missing_report_links'] and frozen['devices_opened'] == 0
    checks.append('Separate original170 bindings and cached57 middleware source verification passed')
    refinement=read(OUT/'next_cycle/handoff_refinement_20261110/verification.json')
    assert refinement['status']=='passed' and not refinement['whole_cycle_or_hardware_success']
    assert refinement['original_source_model_preserved'] and refinement['devices_opened']==0
    checks.append('Separate contact-convergence,9049-goal chain and failed48 dwell checks passed without task/hardware approval')
    whole_folder=OUT/'next_cycle/handoff_refinement_20261110/whole_hand_preflight_v2'
    if (whole_folder/'review.json').exists():
        whole=read(whole_folder/'review.json');unit=read(OUT/'whole_hand_support_unit_verification.json')
        assert readiness['whole_hand_support_preparation']['sha256']==sha(whole_folder/'review.json')
        native_path=whole_folder/'native_summary.json'
        assert readiness['whole_hand_support_preparation']['native_evaluation_pending']==(not native_path.exists())
        assert unit['status']=='passed' and unit['tests_run']==14 and unit['failures']==unit['errors']==0
        assert unit['core_source_sha256']==sha(OUT/'whole_hand_support.py')
        assert unit['test_source_sha256']==sha(OUT/'test_whole_hand_support.py')
        for name,digest in whole['files_sha256'].items():
            if sha(OUT/name)==digest:continue
            assert name=='evaluate_staged_shadow_model.py'
            assert validate_evaluator_revision(digest)['identical_goals_and_bound_state']
        assert len(whole['cached_model_force_checks'])==4
        assert all(not row['external_contact_data_available'] and row['conditional_contact_accounting_only_not_a_new_hold_pass']
                   for row in whole['cached_model_force_checks'])
        assert all(not row['old_result_reclassified'] for row in whole['archived_trial_review'])
        assert whole['archived_failures_only_due_to_legacy_tip_contacts']==0
        stopped=[row for row in whole['archived_trial_review'] if 'stored_endpoint_kinematics' in row]
        assert len(stopped)==5
        for row in stopped:
            k=row['stored_endpoint_kinematics'];state=OUT/row['path'];state=state.parent/'end_state.npz'
            assert sha(state)==k['state_sha256'] and k['displacement_mm']>80
            assert k['upward_mm']<0 and k['upward_velocity_m_s']<0
            assert k['complete_hand_contact_loss_or_outside_hand_not_established_by_this_analysis']
        assert not whole['fresh_native_model_evaluations_executed'] and not whole['hardware_execution_allowed']
        queue=read(whole_folder/'queue_check.json')
        assert queue['analytical_tests']==14 and len(queue['tasks'])==3
        for task in queue['tasks']:assert sha(OUT/task['script'])==task['script_sha256']
        assert not queue['native_dynamics_executed_by_this_check'] and not queue['whole_cycle_ready_for_hardware']
        checks.append('Whole-hand support14 analytical cases and4 cached force sums agree;15 archived trials retain their decisions and5 falling-motion endpoints are not mislabeled as contact-loss proof;3 native tasks remain separate')
        if native_path.exists():
            native=read(native_path)
            assert sha(native_path)==readiness['whole_hand_native_results']['sha256']
            assert native['native_queue_tasks_completed']==3 and native['recorded_ring_geometry_queries']==9
            assert native['user_changed_approval_mode_and_native_execution_succeeded']
            assert native['account_usage_reset_or_credit_purchase_not_inferred']
            assert native['full_hand_saved_state_queries']==6 and native['all_force_frame_checks_passed']
            actual=read(OUT/native['completed_first_rotation_trial']['path'])
            assert sha(OUT/native['completed_first_rotation_trial']['path'])==native['completed_first_rotation_trial']['sha256']
            assert actual['steps']==4118 and not actual['first_rotation_and_hold_passed']
            assert actual['support_mode']=='whole_hand_v1' and actual['terminal_hand_support']['by_geom']=={}
            assert actual['terminal_hand_support']['upward_support_N']==0
            assert native['same_parent_goals_reproduced'] and max(native['end_state_max_abs_difference_from_parent'].values())==0
            for name,digest in native['source_reports_sha256'].items():assert sha(OUT/name)==digest
            assert all(task['result_exists'] for task in queue['tasks'])
            assert not native['whole_cycle_ready_for_hardware'] and not native['hardware_execution_allowed']
            for probe in native['model_feedback_recovery_probes']:
                if not probe['completed']:continue
                path=OUT/probe['path'];r=read(path)
                assert sha(path)==probe['sha256']
                assert not r['complete_cycle_passed'] and not r['hardware_execution_allowed']
                goals=np.load(path.parent/'research_goals.npz',allow_pickle=False)
                assert len(goals['goals'])==r['steps']
                assert np.max(abs(np.diff(np.vstack([goals['start'],goals['goals']]),axis=0)))<=2+1e-9
                assert all(row['prediction_qpos_error']<=1e-8 for row in r['trace'])
                assert r['worst']['self_mm']<=1e-6 and r['worst']['tracking_rad']<=.18
                if probe.get('goal_reference_state'):
                    assert sha(OUT/probe['goal_reference_state'])==probe['goal_reference_sha256']
            diagnosis=native.get('local_feedback_stop_diagnosis')
            if diagnosis:
                path=OUT/diagnosis['path'];assert sha(path)==diagnosis['sha256']
                r=read(path)
                assert r['proposals']==98 and len(r['safe_proposals'])==1
                assert r['rejection_counts']=={'self_mm':97}
                hold=next(row for row in r['rows'] if row['proposal']=='hold')
                assert set(hold['last_guard']['self_pairs_mm'])=={'mf_tip|palm_collision_6'}
                assert hold['last_guard']['self_mm']<.001 and hold['whole_hand']['supported_sample']
                assert r['original_guards_and_limits_retained'] and not r['hardware_execution_allowed']
            coupled_path=OUT/'next_cycle/whole_hand_recovery_cp1700_w0.15_coupled/review.json'
            if coupled_path.exists():
                coupled=read(coupled_path)
                baseline=np.load(OUT/'next_cycle/whole_hand_recovery_cp1700_w0.15/research_goals.npz',allow_pickle=False)['goals']
                improved=np.load(coupled_path.parent/'research_goals.npz',allow_pickle=False)['goals']
                assert np.array_equal(improved[:len(baseline)],baseline)
                assert coupled['coupled_fallback_enabled'] and coupled['steps']>len(baseline)
                assert coupled['coupled_fallback_events'][0]['after_goals']==len(baseline)
                assert coupled['coupled_fallback_events'][0]['valid']>0
            checks.append('Manual approval enabled actual9-pose,6-force-query and4118-goal native results; full-hand contact loss is now measured in the model; local feedback preserves2tick/native guards and no hardware success')
            auto=native.get('automatic_continuation',{})
            if auto.get('studies'):
                assert auto['user_requested_automatic_approval'] and auto['new_SAC_training_steps']==0
                assert not auto['hardware_execution_allowed']
                for study in auto['studies'].values():assert sha(OUT/study['path'])==study['sha256']
                geometry=read(OUT/'convex_pair_gap_verification.json')
                assert geometry['status']=='passed' and geometry['tests_run']==7
                assert geometry['geometry_source_sha256']==sha(OUT/'convex_pair_gap.py')
                assert geometry['test_source_sha256']==sha(OUT/'test_convex_pair_gap.py')
                if 'SAC_gain_sensitivity' in auto['studies']:
                    study=read(OUT/auto['studies']['SAC_gain_sensitivity']['path'])
                    assert [row['gain_rad'] for row in study['rows']]==[.01,.03,.05]
                    assert study['original_gain_with_projection_goals_equal_previous'] and study['new_SAC_training_steps']==0
                    assert len({row['actor_sha256'] for row in study['rows']})==1
                    for row in study['rows']:
                        path=OUT/row['path'];assert sha(path)==row['sha256']
                        result=read(path);scale=result['actor_scaling_research']
                        assert scale['applied_residual_rad']==row['gain_rad'] and not scale['retrained_for_this_gain']
                        assert result['no_state_reset_after_initial_placement'] and not result['hardware_execution_allowed']
                        assert sha(path.parent/'source_evaluate_staged_shadow_model.py')==sha(OUT/'evaluate_staged_shadow_model.py')
                        commands=np.load(path.parent/'research_trace.npz',allow_pickle=False)['goals']
                        assert len(commands)==result['steps'] and np.max(abs(np.diff(commands,axis=0)))<=2
                if 'palm_contact_convergence' in auto['studies']:
                    study=read(OUT/auto['studies']['palm_contact_convergence']['path'])
                    assert study['states']==9 and len(study['rows'])==36
                    assert {row['iterations'] for row in study['rows']}=={50,100,200,500}
                    assert not study['positive_convex_gap_with_negative_native_contact']
                    assert study['geometry_source_sha256']==sha(OUT/'convex_pair_gap.py')
                    for name,digest in study['source_hashes'].items():assert sha(OUT/name)==digest
                if 'RF_endpoint_current_model_holds' in auto['studies']:
                    study=read(OUT/auto['studies']['RF_endpoint_current_model_holds']['path'])
                    assert len(study['rows'])==4
                    for row in study['rows']:
                        assert row['initialized_RF_hold_passed'] and row['steps_1ms']==4050 and not row['stop']
                        assert row['RF_loaded_fraction']>.8 and row['whole_hand_supported_fraction']>.8
                        assert not row['initialized_work_grip_fully_certified']
                        assert row['certificate']['queries']==1 and row['upper_certificate_scope'].startswith('Final pose only')
                if 'RF_upper_blockers' in auto['studies']:
                    study=read(OUT/auto['studies']['RF_upper_blockers']['path'])
                    assert sha(OUT/study['source'])==study['source_sha256']
                    witnesses=[row for row in study['rows'] if row['witness_within_required_margin']]
                    assert {row['geom'] for row in witnesses}=={'if_tip','rf_tip','cadtip_mf_rf_rf_ds_0','cadtip_mf_rf_rf_ds_1'}
                    assert all(row['constraint_violation_mm_or_mm2']<1e-7 and row['feasible_pair_distance_upper_mm']<.3 for row in witnesses)
                if 'RF_clear_work_grip_joined_suffix' in auto['studies']:
                    study_path=OUT/auto['studies']['RF_clear_work_grip_joined_suffix']['path'];study=read(study_path)
                    assert study['conditional_clear_work_grip_suffix_passed'] and study['steps']==392 and study['stage_goals']==[90,302]
                    assert not study['stop'] and study['actual_state_resets_after_start']==0
                    assert study['final_hold_samples']==study['certified_samples']==4050
                    assert study['upper_certificate_minimum_lower_mm']>.3 and study['whole_hand_support_fraction']>.8
                    assert study['final_global90_error_deg']<5 and study['common_station_error_mm']<2
                    assert study['hold_angle_drift_deg']<3 and study['hold_center_drift_mm']<2
                    assert all(row['exact_bound_state_equal'] and max(row['state_max_abs_difference'].values())==0 for row in study['seams'])
                    assert study['initial_RF_endpoint_itself_was_initialized_not_reached_from_initial_placement']
                    assert study['first_rotation_from_initial_placement_or_next_rotation_not_certified'] and not study['hardware_execution_allowed']
                    for name,digest in study['source_reports_sha256'].items():assert sha(OUT/name)==digest
                    assert sha(OUT/study['source'])==study['source_sha256']
                    commands=np.load(study_path.parent/'research_goals.npz',allow_pickle=False)
                    assert len(commands['goals'])==392 and np.max(abs(np.diff(np.vstack([commands['start'],commands['goals']]),axis=0)))<=2
                    assert study['worst']['self_mm']<=1e-6 and study['worst']['tracking_rad']<=.18 and study['worst']['range_rad']<=1e-6
                    assert not any(name.startswith('rf_') for name in study['terminal_support']['by_geom'])
                    clear_stage=read(OUT/auto['studies']['RF_index_clearance_margin2']['path'])
                    first=clear_stage['dynamic_trials'][0]
                    assert first['initialized_IF_clear_whole_hand_hold_passed'] and first['IF_clear_samples']==4050
                    assert first['IF_certificate_minimum_lower_mm']>1 and not first['stop']
                    visualization=read(study_path.parent.parent/'RF_clear_work_grip_evidence/manifest.json')
                    assert visualization['scientific_model_renders_not_photos'] and len(visualization['images'])==6
                    assert visualization['display']['physics_properties_unchanged']
                    for item in visualization['images']+visualization['videos']:assert sha(OUT/item['source'])==item['source_sha256']
                    checks.append('392 conditional IF/RF clearance goals replay with no reset and exact seams;4050 full-hand support and sweep certificates pass, RF contact is absent, initial-placement/next-turn/hardware claims remain false')
                if 'clear_work_grip_load_hypotheses' in auto['studies']:
                    study=read(OUT/auto['studies']['clear_work_grip_load_hypotheses']['path'])
                    assert sha(OUT/study['source'])==study['source_sha256']
                    assert len(study['cases'])==11 and sum(row['conditional_hold_passed'] for row in study['cases'])==7
                    assert study['applied_moment_is_hypothesis_not_measured_left_hand_load']
                    assert study['model_torque_is_not_calibrated_to_450mA'] and not study['hardware_execution_allowed']
                    for row in study['cases']:
                        assert row['original_guards_and_joint_limits_preserved'] and row['only_sliding_friction_entry_changed']
                        if row['conditional_hold_passed']:
                            assert row['samples']==row['certified_window_samples']==4050 and not row['stop']
                            assert row['upper_minimum_lower_mm']>.3 and row['whole_hand_supported_fraction']>.8
                        if abs(row['assumed_constant_moment_about_up_Nm'])>=.003:assert not row['conditional_hold_passed']
                    checks.append('11 conditional friction/moment holds retain7 passes and4 failures; simulated discrete loads are not measured left-hand capacity or450mA calibration')
                checks.append('Automatic offline studies retain unchanged SAC weights across3 gains,7 analytic geometry checks,36 CCD/convex comparisons and4 initialized RF holds; negative certificate bounds are not penetration depths and no arrival/full-cycle success is invented')
    files = []
    paths = set(OUT.glob('*.py')) | set(OUT.glob('*.md')) | set(OUT.glob('*.json'))
    paths |= set(OUT.glob('review/*'))
    for pattern in ('**/result.json', '**/review.json', '**/manifest.json', '**/settings.json',
                    '**/source_*.py', '**/*_source.py', '**/actor.npz', '**/research_goals.npz',
                    'shadow_bundle_nominal_only/*', 'sac_*/checkpoints/**/*',
                    'next_cycle/**/*.py','next_cycle/**/*.md','next_cycle/**/*.json',
                    'next_cycle/**/examples.npz','next_cycle/**/proposal*.npz',
                    'next_cycle/**/end_state.npz','next_cycle/**/state_pool.npz','next_cycle/**/model*.mjb',
                    'next_cycle/**/research_trace.npz','next_cycle/**/state*_baseline.npz',
                     'next_cycle/**/*_proposal.npz',
                     'next_cycle/**/phase*_end_state.npz','next_cycle/**/state.npz','next_cycle/**/goals.npz','next_cycle/**/trajectory.npz',
                     'next_cycle/handoff_refinement_20261110/RF_clear_work_grip_joined_suffix/trajectory.npz',
                     'next_cycle/handoff_refinement_20261110/RF_clear_work_grip_evidence/*',
                      'next_cycle/handoff_refinement_20261110/opposing_grip_review_20261110/*',
                       'next_cycle/handoff_refinement_20261110/ring_front_thumb_progress_20261111/*',
                       'next_cycle/handoff_refinement_20261110/user_ring_photo_reference_20261110/*',
                       'next_cycle/handoff_refinement_20261110/first_to_next_contact_bridge_20261111/*',
                       'next_cycle/handoff_refinement_20261110/MF_TH_interference_diagram_20261111/*',
                       'next_cycle/**/controller_state.npz',
                       'next_cycle/**/rotation_arrival_*.npz',
                       'next_cycle/**/waypoint_reference_only.npz','next_cycle/**/static_motor_waypoints.npz',
                       'next_cycle/handoff_refinement_20261110/robust_cycle_preparation_20261111/**/*.npz',
                       'next_cycle/handoff_refinement_20261110/reposition_then_second_rotation_20261111/**/*',
                       'next_cycle/handoff_refinement_20261110/robust_mu025_*/*.npz',
                      'next_cycle/**/goal_reference_only.npz',
                    'next_cycle/**/*.html','next_cycle/staged_shadow_bundle*/*'):
        paths |= set(OUT.glob(pattern))
    paths |= {ROOT / 'render_policy_failure_explanation_20261109.py',
              ROOT / 'saved_policy_observation_provider_20261109.py',
              ROOT / 'right_hand_rotation_20261106/middle_clearance_probe/evidence/model.mjb',
              ROOT / 'cloud_handoff_20261106/CLOUD_START_PROMPT.md',
              ROOT / 'professor_progress_20261015.md',
              ROOT / 'right_hand_rotation_20261109/policy_audit/STATUS.md',
              ROOT / 'right_hand_rotation_20261109/policy_audit/explanation/index.html', frozen_path}
    paths.add(ROOT/'ring_reference_readonly_20261109_124719_945552/result.json')
    paths.add(ROOT/'leap_whole_cycle_shadow_check.cmd')
    paths |= {ROOT/name for name in calibration['source_hashes']}
    paths |= {ROOT/r['path'] for r in calibration['camera_intrinsic_candidates']}
    for path in sorted(paths):
        if not path.is_file() or path.name in ('artifact_manifest.json', 'artifact_verification.json'):
            continue
        files.append(dict(path=path.relative_to(ROOT).as_posix(), bytes=path.stat().st_size, sha256=sha(path)))
    inventory = dict(format='prehardware_policy_cycle_research_inventory_v2', date='2026-11-11',
                      files=files, new_SAC_environment_steps=summary['new_training_steps'], first_stage_only=True,
                      SAC_step_count_is_completed_requested_budget_not_exact_trainer_counter=True,
                      old_CCD10_successes_not_transferred_to_CCD50=True,
                      connected_model_cases_are_training_cases=True,
                     saved_teacher_suffix_is_not_SAC=True, next_rotation_passed=False,
                     physical_validation_completed=False, distillation_completed=False,
                     hardware_execution_allowed=False, devices_opened=0)
    if whole_cycle.exists():
        latest=read(whole_cycle)
        inventory.update(first_stage_only=not latest['nominal_two_turn_reached_locations_passed'],
                         nominal_two_rotations_and_reached_location_holds_passed=latest['nominal_two_turn_reached_locations_passed'],
                         nominal_two_turn_replay_goals=latest['nominal_two_turn_replay_steps'],
                         next_rotation_passed_scope='Existing common-station2mm task and required robustness; nominal reached-location result is separate',
                         same_first_station_two_turn_cycle_passed=latest['nominal_same_first_station_cycle_passed'],
                         finite_two_turn_perturbations=latest['finite_perturbation_conditions'],
                         finite_two_turn_perturbation_passes=latest['finite_perturbation_passes'])
    if entry.exists():
        additional=read(entry)
        inventory.update(additional_mu025_two_turn_reached_location_holds_passed=additional.get('full_two_turn_two_hold_task_passed',False),
                         additional_mu025_small_error_robustness_passed=additional.get('new_cycle_small_error_robustness_passed',False),
                         additional_model_path_is_stored_teacher_goals_not_new_adaptive_hardware_policy=True,
                         additional_model_cycle_report_sha256=sha(entry),
                         additional_model_cycle_left_hand_reach_load_and_registration_unverified=True)
    (OUT / 'artifact_manifest.json').write_text(json.dumps(inventory, ensure_ascii=False, indent=2), encoding='utf-8')
    result = dict(status='passed', checks=checks, source_and_asset_files=len(files),
                  manifest_sha256=sha(OUT / 'artifact_manifest.json'), devices_opened=0,
                  verification_is_not_whole_cycle_or_hardware_success=True)
    (OUT / 'artifact_verification.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
