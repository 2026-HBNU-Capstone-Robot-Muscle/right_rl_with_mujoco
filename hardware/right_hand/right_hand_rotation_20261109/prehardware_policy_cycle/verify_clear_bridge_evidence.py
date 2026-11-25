"""Cross-check executed bridge goals, source chains and explicitly limited claims."""
from pathlib import Path
import hashlib
import json
import numpy as np

OUT=Path(__file__).resolve().parent;BASE=OUT/'next_cycle/handoff_refinement_20261110'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def read(path):return json.loads(path.read_text(encoding='utf-8'))


def goals(path):
    data=dict(np.load(path,allow_pickle=False));joined=np.vstack([data['start'],data['goals']])
    assert np.isfinite(joined).all() and np.max(abs(np.diff(joined,axis=0)))<=2
    return data


def verify():
    checks=[];unit=read(OUT/'planning_pair_gap_verification.json')
    assert unit['status']=='passed' and unit['tests_run']==7 and unit['failures']==unit['errors']==0
    assert unit['source_sha256']==sha(OUT/'planning_pair_gap.py') and unit['test_source_sha256']==sha(OUT/'test_planning_pair_gap.py')
    benchmark=read(BASE/'planning_pair_gap_benchmark.json')
    assert len(benchmark['rows'])==2 and benchmark['bounded_source_sha256']==unit['source_sha256']
    assert all(not row['native_guard'] for row in benchmark['rows'])
    checks.append('Seven analytical planning-gap cases pass; candidate cost and exact native/upper guards remain separate')
    for tag,count in (('first90_finish_checkpoint',732),('next90_recenter_checkpoint',600)):
        folder=BASE/tag;manifest=read(folder/'manifest.json')
        assert manifest['executed_goals']==count and manifest['state_control_time_consistency_error_seconds']<1e-6
        assert sha(folder/'state.npz')==manifest['state_sha256'] and sha(folder/'goals.npz')==manifest['goals_sha256']
        assert sha(folder/'source_plan_clear_work_grip_bridge.py')==manifest['source_code_sha256']
        saved=goals(folder/'goals.npz');assert len(saved['goals'])==count
        parent=OUT/manifest['parent'];parent_goals=goals(parent/'research_goals.npz' if (parent/'research_goals.npz').exists() else parent/'checkpoint_goals.npz')
        assert np.array_equal(saved['goals'],parent_goals['goals'][:count])
    checks.append('Executed732/600-goal branch snapshots match immutable bytes, parent commands and model time')
    chain_folder=BASE/'clear_grip_next5_canonical_joined_chain';chain=read(chain_folder/'review.json')
    assert chain['steps']==804 and chain['boundaries']==[392,526,574,632,804] and chain['actual_state_resets_after_start']==0
    assert chain['conditional_canonical_station_task_passed'] and not chain['next90_passed']
    assert not chain['full_initial_placement_two_turn_cycle_passed'] and chain['initial_RF_endpoint_was_initialized_not_reached_from_initial_placement']
    assert not chain['hardware_execution_allowed'] and chain['model_based_teacher_not_SAC'] and chain['new_SAC_training_steps']==0
    assert chain['final_hold_samples']==chain['certified_samples']==4050 and chain['upper_certificate_min_lower_mm']>.3
    assert chain['whole_hand_support_fraction']>.8 and chain['final_orientation_error_deg']<1 and chain['canonical_station_error_mm']<2
    assert 4<chain['additional_yaw_after_clear_grip_deg']<6 and chain['hold_angle_drift_deg']<3 and chain['hold_center_drift_mm']<2
    assert all(row['all_bound_state_exact'] and max(row['max_abs_by_state'].values())==0 for row in chain['seams'])
    assert sha(OUT/chain['source'])==chain['source_sha256']
    assembled=[]
    for stage in chain['stages']:
        folder=OUT/stage['path'];assert sha(folder/'review.json')==stage['report_sha256'] and sha(folder/'research_goals.npz')==stage['goals_sha256']
        assembled.extend(goals(folder/'research_goals.npz')['goals'])
    assert np.array_equal(np.asarray(assembled),goals(chain_folder/'research_goals.npz')['goals'])
    checks.append('804 connected goals reproduce five exact state seams; final4050 support/clearance states and95-degree/canonical task pass without initial-placement or next90 claims')
    actor=read(BASE/'original108_governed_bridge_probes/review.json')
    assert len(actor['rows'])==6 and actor['actual_joint_observations_quantized_to_encoder_ticks'] and not actor['hardware_execution_allowed']
    for row in actor['rows']:
        assert row['stop']==['self_mm'] and row['complete_turn_and_final_hold_not_certified']
        assert len(goals(OUT/row['path']/'research_goals.npz')['goals'])==row['steps']
    checks.append('Original108 actor has six integer-observation governed dynamic failures, separate from local110 SAC/reference policy')
    regrasp=read(BASE/'actual_cube_MF_TH_regrasp_global/review.json')
    assert len(regrasp['trials'])==24 and not regrasp['regrasp_passed']
    for row in regrasp['trials']:
        assert not row['stop'] and row['hold_samples']==4050 and row['whole_hand_support_fraction']>.8
        assert not row['MF_TH_regrasp_for_translation_passed'] and row['MF_loaded_fraction']==row['TH_loaded_fraction']==0
        assert row['worst']['self_mm']<=1e-6 and row['worst']['tracking_rad']<=.18
        assert len(goals(OUT/row['path']/'research_goals.npz')['goals'])==row['steps']
    checks.append('Twenty-four recontact paths retain whole-hand support but fail paired contact load; no candidate is relabeled a stable pinch')
    result=dict(status='passed',checks=checks,devices_opened=0,whole_initial_placement_two_turn_or_hardware_not_certified=True)
    (BASE/'bridge_progress_20261110/verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    return result


if __name__=='__main__':print(json.dumps(verify(),ensure_ascii=False),flush=True)
