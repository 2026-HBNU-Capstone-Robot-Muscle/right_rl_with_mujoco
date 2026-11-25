"""Local handoff research evidence, with initialized endpoints kept separate."""
from pathlib import Path
import json
import hashlib
import html

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
FOLDER=OUT/'next_cycle/handoff_refinement_20261110'
BEGIN='<!-- rf_handoff_refinement_20261110 -->'
END='<!-- rf_handoff_refinement_20261110_end -->'


def load(path):return json.loads(path.read_text(encoding='utf-8'))
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def insert(path,body,after=None):
    source=path.read_text(encoding='utf-8')
    if BEGIN in source:
        start=source.index(BEGIN);finish=source.index(END,start)+len(END)
        source=source[:start]+source[finish:].lstrip('\n')
    block=BEGIN+'\n'+body.rstrip()+'\n'+END+'\n\n'
    if after:
        i=source.index(after)+len(after);source=source[:i]+'\n\n'+block+source[i:].lstrip('\n')
    else:source=block+source
    path.write_text(source,encoding='utf-8')


def main():
    endpoint_path=FOLDER/'endpoint_screen/review.json';ep=load(endpoint_path)
    audit=load(FOLDER/'support_audit.json');branches=[]
    for path in sorted(OUT.glob('next_cycle/handoff_*/review.json')):
        r=load(path)
        if r.get('status')!='force_feedback_teacher_for_cooperative_contact_bridge':continue
        branches.append(dict(path=path.relative_to(OUT).as_posix(),sha256=sha(path),steps=r['steps'],stop=r['stop'],
            passed=r['temporary_third_support_passed'],final=r['final'],worst=r['worst'],
            final_support_fraction=r['final_support_fraction'],source_state=r.get('source_state_path'),
            source_sha256=r['source_sha256'],reference_path=r.get('reference_path'),reference_sha256=r['planned_path_sha256'],
            resume_state=r.get('resume_state'),resume_state_sha256=r.get('resume_state_sha256'),
            restore_alignment=r.get('resume_state_alignment_max_abs'),is_SAC=False))
    plans=[]
    for name in ('rrt_rf_seed1_equilibrium1','rrt_rf_seed3_equilibrium3','rrt_rf_seed5_integer_equilibrium3'):
        path=OUT/'next_cycle'/name/'review.json'
        if not path.exists():continue
        r=load(path);d=r.get('dynamic')
        plans.append(dict(path=path.relative_to(OUT).as_posix(),sha256=sha(path),plan_found=r['collision_plan_found'],
                          source=r.get('source_state_path'),dynamic=d))
    expected=('handoff_wrench_free_v1','handoff_wrench_root_fixed_v1','handoff_equilibrium1_feedback_v2',
              'handoff_equilibrium1_integer_start_v2','handoff_integer_endpoint_close_v3',
              'handoff_integer_endpoint_close_v4','handoff_integer_endpoint_close_v5','handoff_integer_load_first_v6',
              'handoff_integer_load_first_v7')
    pending=[name for name in expected if not (OUT/'next_cycle'/name/'review.json').exists()]
    endpoint_passes=sum(r['passed'] for r in ep['qualified_endpoints'])
    bridge=next(r for r in plans if 'seed5_' in r['path'])['dynamic']
    report=dict(date='2026-11-10',support_force_convention_verified_states=len(audit['states']),
        maximum_constraint_force_error=max(r['wrench']['constraint_force_error'] for r in audit['states']),
        endpoint_screen_trials=len(ep['screen_trials']),screen_passes=sum(r['viable_screen'] for r in ep['screen_trials']),
        initialized_endpoint_hold_tests=len(ep['qualified_endpoints']),initialized_endpoint_holds_passed=endpoint_passes,
        endpoint_tests_are_not_reached_trajectories=True,endpoint_review_sha256=sha(endpoint_path),
        motion_plans=plans,feedback_branches=branches,pending_final_reports=pending,
        third_support_motion_passed=any(r['passed'] for r in branches),
        second90_rotation_passed=False,whole_cycle_ready_for_hardware=False,
        new_SAC_environment_steps_this_refinement=0,previous_completed_SAC_environment_steps=60000,
        frozen_original_execution_files_not_edited=True,devices_opened=0,remote_writes=0,monitoring_ui_changes=0)
    collision=load(FOLDER/'collision_convergence/manifest.json')
    replay50=load(FOLDER/'ccd_replay50/review.json');replay100=load(FOLDER/'ccd_replay100/review.json')
    fresh_path=OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_active_ccd50/review.json'
    fresh=load(fresh_path)
    chain=load(FOLDER/'connected_chain_through_v6/review.json')
    report['collision_convergence']=dict(manifest_path='collision_convergence/manifest.json',
        source_model_sha256=collision['source_model_sha256'],variant_model_sha256=collision['variant_model_sha256'],
        only_option_change=collision['changed_options'],unchanged_array_fields=len(collision['unchanged_model_array_fields']),
        original_native_CCD_was_already_enabled=True,static_comparison_path='CCD_comparison.json',
        same_command_goals=replay50['goals_completed'],iterations50_vs100_end_max_abs=replay100['end_difference_from50'],
        fresh_parent_policy_path=fresh_path.relative_to(OUT).as_posix(),
        fresh_parent_policy_steps=fresh['steps'],fresh_parent_policy_passed=fresh['first_rotation_and_hold_passed'],
        fresh_parent_policy_stop=fresh['stop'],fresh_parent_policy_final=fresh['final'],
        old_CCD10_successes_not_transferred_to_CCD50=True)
    report['continuous_chain']=dict(path='connected_chain_through_v6/review.json',goals=chain['goals'],
        seam_max_abs=max(v for s in chain['seams'] for v in s['max_abs'].values()),
        dynamics_reproduction=chain['continuous_dynamics_reproduction_passed'],
        final_error_deg=chain['first_to_handoff_target90_error_deg'],
        final_RF_TH_base_fraction=chain['rf_th_base_support_fraction'],task_success=False)
    run=OUT/'sac_ccd50_repair_10k'
    trained=load(run/'result.json') if (run/'result.json').exists() else None
    progress=load(run/'progress.json') if (run/'progress.json').exists() else None
    report['SAC_CCD50_repair']=dict(path='sac_ccd50_repair_10k',completed=bool(trained),
        planned_steps=10000,completed_requested_steps=trained['steps'] if trained else 0,
        trainer_reported_steps=progress['steps'] if progress else None,
        checkpoint_restore='sac_current_contract_v4_20k/checkpoints/000000010256',
        reset_windows_are_actual_CCD50_states=True,reset_indices=[3000,3400,3600],
        learning_windows_are_not_full_cycle_success=True)
    report['new_SAC_environment_steps_this_refinement']=trained['steps'] if trained else 0
    followups=[]
    for name in ('sac_ccd50_repair_10k','sac_ccd50_recovery_long_10k','sac_ccd50_early_recovery_10k'):
        f=OUT/name
        if not (f/'settings.json').exists():continue
        settings=load(f/'settings.json');done=load(f/'result.json') if (f/'result.json').exists() else None
        counter=load(f/'progress.json') if (f/'progress.json').exists() else None
        followups.append(dict(run=name,completed=bool(done),completed_requested_steps=done['steps'] if done else 0,
            trainer_reported_steps=counter['steps'] if counter else None,
            horizon=settings.get('horizon',512),reset_indices=settings['reset_indices'],
            positive_valid_step_reward='valid_step_reward_lower_bound' in settings,
            learning_windows_are_not_full_cycle_success=True))
    report['SAC_CCD50_followups']=followups
    report['new_SAC_environment_steps_this_refinement']=sum(r['completed_requested_steps'] for r in followups)
    earlier=load(OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_schema_v2_confirmation/review.json')
    by_step={r['step']:r for r in earlier['trace']}
    differences=[dict(step=r['step'],seconds=r['step']*.075,
                     orientation_error_difference_deg=abs(r['error_deg']-by_step[r['step']]['error_deg']))
        for r in fresh['trace'] if r['step'] in by_step and abs(r['error_deg']-by_step[r['step']]['error_deg'])>2]
    report['first_recorded_orientation_error_difference_above2deg']=differences[0] if differences else None
    negative=[]
    for pattern in ('gravity_*/review.json','lag_interpolation/review.json','fixed_grip_search*/review.json',
                    'stable_load_search/review.json','dwell_cutoff_search/review.json'):
        for path in sorted(FOLDER.glob(pattern)):
            negative.append(dict(path=path.relative_to(FOLDER).as_posix(),sha256=sha(path)))
    report['additional_negative_diagnostics']=negative
    repair_trials=[]
    for path in sorted(OUT.glob('next_cycle/staged_model_*ccd50repair*/review.json')):
        r=load(path);repair_trials.append(dict(path=path.relative_to(OUT).as_posix(),steps=r['steps'],
            passed=r['first_rotation_and_hold_passed'],stop=r['stop'],final=r['final']))
    report['fresh_repair_trials']=repair_trials
    sampling_path=FOLDER/'observation_timing_audit.json'
    if sampling_path.exists():
        sampling=load(sampling_path)
        report['observation_sampling_audit']=dict(path='observation_timing_audit.json',
            states=len(sampling['rows']),all_compared_next_goals_match=sampling['all_compared_next_goals_match'],
            maximum_cube_pose_cache_difference_mm=max(r['cube_translation_cache_difference_mm'] for r in sampling['rows']),
            scope=sampling['scope'])
    report['next_required_evidence']=[
        'First rotation and stable working hold from initial placement in the CCD50 model',
        'Reachable load transfer to the ring finger, not an initialized static endpoint',
        'Second complete90-degree turn and hold with no state reset',
        'Placement, sensing and actuation uncertainty trials before whole-cycle physical testing']
    report['policy_reuse_decision']=dict(path='POLICY_REUSE_DECISION.md',
        current_repair_exports_adopted=False,
        all_completed_initial_placement_repair_trials_failed=bool(repair_trials) and not any(r['passed'] for r in repair_trials),
        residual_authority_audit_path='action_authority_audit.json',
        proposal_offset_rad=.01,whole_cycle_distillation_completed=False)
    sensitivity=FOLDER/'small_joint_response/answer_summary.json'
    if sensitivity.exists():
        report['small_joint_response_and_recorded_ring_reference']=dict(path='small_joint_response/STATUS.md',
            summary_path='small_joint_response/answer_summary.json',sha256=sha(sensitivity),
            small_residual_radius_not_established_as_primary_failure_cause=True)
    (FOLDER/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    lines=[]
    for r in branches:
        f=r['final'] or {}
        lines.append(f"- [{r['path'].split('/')[-2]}](../../{r['path']}): {r['steps']}목표, 중단`{r['stop']}`, 임시 약지 지지{'통과' if r['passed'] else '미달'}, 방향 오차{f.get('orientation_error_deg',0.):.3f}도, 위치 오차{f.get('center_error_mm',0.):.3f}mm.")
    convergence=f'''## 접촉 계산 수렴과 정책 보정 — 가장 최근 판정

기존 접촉 계산10회에서는 같은 형상의 약지 끝 약0.020mm 겹침과 검지 끝 약0.0198mm 겹침을 거리 계산과 독립 볼록 형상이 확인했지만 실제 접촉 목록에는 두 손끝이 없었다.50·100·200회에서는 두 접촉을 일관되게 잡았다. 기존 모델도 native CCD는 켜져 있었고, 처음 진단에서 비트 의미를 잘못 붙인 표시는 `CCD_comparison.json`에 정정했다. 형상을 빼거나 접촉 마스크를 바꾸지 않았다.

원본 모델을 보존하고 접촉 계산 상한만10→50으로 바꾼 별도 모델을 만들었다. 모델 배열{report['collision_convergence']['unchanged_array_fields']}개가 동일하며 관절 범위·힘 한도·PID·마찰·형상과 동적 중단 기준은 같다. [MuJoCo 공식 문서의 CCD 반복 설정](https://mujoco.readthedocs.io/en/stable/XMLreference.html#option-ccd-iterations)을 참고했다. 고정 명령4118개를50회와100회로 재생한 최종 qpos/qvel/ctrl/act/warmstart 차이는 전부0이다. 이것은 이 명령열의 수치 비교이며 모든 자세의 수렴이나 실물 정확도를 보장하지 않는다.

**첫 회전부터 다시 실행한 기존 정책은 새 모델에서{fresh['steps']}목표 후 큐브80mm 이탈로 중단됐다.** 방향 오차{fresh['final']['error_deg']:.3f}도다. 이전10회 모델의6956목표·약41초 유지 성공은 그대로 보존하지만 수렴 비교를 통과한50회 모델의 성공으로 이전하지 않는다. 따라서 첫 회전·유지까지 포함해 다시 검증해야 하며, 다음 회전이 준비됐다는 판정은 없다.

실제50회 모델 상태에서 기존 SAC actor를 복원한 보정 실험을 수행했다. 이 후속에서 완료한 요청 학습 분량 합계는{report['new_SAC_environment_steps_this_refinement']:,}스텝이며 실제 trainer의 묶음 진행 번호는 각 기록에 별도로 남긴다. 세부: {json.dumps(followups,ensure_ascii=False)}. 초기화가 있는 학습 창의 성능을 연속 회전 성공으로 세지 않는다.

짧은 학습 창은 실제 실패4118목표 전에 끝나는 조건이 있었고 평균 점수가 개선돼도 중간5256/최종10000정책의 독립 초기 배치 평가는 각각4267/4243목표에서 큐브를 놓쳤다. 원래 기록을 보존했다. 이후 모든 창이 실패 지점을 지나도록1200주기로 늘리고, 유효한 주기는 항상 양의 보상이며 실제 중단 시−20을 주도록 바꿨다. 긴 음의 보상을 피하려고 조기 종료가 유리해질 수 있는 구조를 줄이는 변경이며 정책이 실제로 그 편법을 배웠다고 입증한 것은 아니다. 동적 중단 기준과 성공 판정은 같다.

또한 저장 제어 주기 표본에서 첫90도 오차 차이2도 초과를 확인한 구간은 {json.dumps(report['first_recorded_orientation_error_difference_above2deg'],ensure_ascii=False)}다. 이는 최초 물리 차이 시점을 확정한 것이 아니라100주기 간격 지표 비교다. 뒷부분만 바꾸는 정책의 한계를 확인하기 위해 실제 초기50회 모델 상태37개를3600주기까지 재현했고 원래 상태와 차이0을 확인했다.600/900/1200/1600/2000/2600/3000/3400에서 시작하는 별도 보정도 비교한다. 각 최종 정책은 초기 배치부터 재평가한다.

독립 초기 배치부터 다시 실행한 결과: {json.dumps(repair_trials,ensure_ascii=False)}.

입력 표본 시점도 따로 점검했다. 세 상태에서 최신 적분 자세와 native 캐시 자세의 차이를 비교했고, 검사한 정책들의 다음 관절 목표는 모두 같았다. 기록은 `observation_timing_audit.json`이다. 세 번의 단일 주기 비교이며 전체 관측 공급기의 동등성이나 실제 카메라 지연 검증으로 확대하지 않는다.

## 기존 정책의 재사용 판정과 보정 범위

완료한 추가 SAC3회·요청 분량3만 스텝의 중간/최종 정책4개를 초기 배치부터 다시 시험했다. 모두 첫 회전·유지 미달이라 실물 실행 정책으로 채택하지 않았다. 총 완료 요청 분량은 이전6만+이번3만=9만이며 실제 trainer 최종 진행 번호도 각3회 모두10000으로 기록됐다. 학습 창 점수 개선을 전체 성공으로 바꾸지 않는다.

이 SAC는 기준 궤적 주위±0.01rad, 약±0.573도/±6.519틱의 목표를 제안하는 잔차 정책이다. 실행 목표의 주기 간격은 여전히 최대2틱이고75ms다. 지령 속도 제한 때문에 실제 지령이 제안 범위 밖에서 잠시 따라올 수 있다는 점과 제안 범위를 구분한다. 별도로 지지가 됐던 초기화 약지 후보와 기존 첫 파지의 약지 네 지령 차이는 약671/−518/−1127/585틱이다. 그 후보를 최소 필수 이동량이라고 입증한 것은 아니지만 현재의 작은 잔차 제안만으로 해당 재파지를 직접 지시하는 구조는 아니다.

원본108차원 정책, 현재110차원 잔차 정책, 증분 재파지 제어를 서로 같은 정책으로 부르지 않는다. 기존 정책을 전부 폐기해야 한다거나 새 학습만이 유일한 해법이라고 결론내리지 않는다. 다음 검토는 수렴 조건의 기준 동작/접촉 모드와 잔차 제어 범위를 함께 맞추는 문제다. 실패한 동작을 증류하지 않는다. [정책 재사용 판정](POLICY_REUSE_DECISION.md)과 [행동 범위 계산](action_authority_audit.json)을 따른다.

## 상태를 초기화하지 않은 연결과 추가 음성 결과

기존10회 모델에서는 초기 배치→실시간 관측 SAC/학습 보정6956목표→저장한 모델 탐색 명령2093목표, 총9049목표를 상태 초기화 없이 재현했다. 여섯 경계의 물리 상태 차이는0이고 동적 기준을 통과했다. 하지만 최종90도 오차14.066도, 위치 오차3.316mm, 마지막 약지·엄지·검지 뿌리 지지 비율0.0988%로 작업 목표는 실패했다. 뒤쪽은 저장 탐색 명령이며 전체 폐루프 학습 정책으로 부르지 않는다.

잠깐 세 지점이 힘을 받는 구간도10.05초 유지에서 살아남지 못했다. 유지 후보48개 비교와 별도 안정 하중 탐색, 모델 중력 보상 세 변형도 성공으로 채택하지 않았다. 검지/엄지를 고정한 약지 후보 탐색2회도 독립 형상 검사와 실제 접촉 조건을 함께 통과하지 못했다. 이런 결과가 모든 재파지의 불가능성을 뜻하지 않는다.

'''
    status=f'''# 약지 하중 인계 후속: 2026-11-10

사용자의 “전체 동작 준비가 아니면 계속” 요청을 이어 수행했다. **전체 회전→유지→다음 회전→유지의 실물 준비 완료가 아니다.** 원본170개와 과거 결과를 보존한다. 접촉 계산 상한을 높인 별도 모델에서 첫 회전부터 재검증하고 기존 SAC를 보정하는 단계다.

{convergence}

## 새로 확인한 점

- 저장 모델 상태4개의 접촉력을 일반화 구속력과 대조했다. 최대 차이{report['maximum_constraint_force_error']:.3g}로 좌표계와 부호를 확인했다. 큐브 모델 무게는0.981N이고 대부분을 검지 뿌리가 받친다. 실제 큐브 하중 측정은 아니다.
- 약지 접촉 후보의 끝 자세 자체를 따로 시험했다.81개 모델 압력 지령 비교 중53개가750ms 선별 조건을 통과했다.4개를10.05초 자유 동역학으로 이어 검사했고{endpoint_passes}개가 마지막4050상태의 약지·엄지·검지 뿌리 힘과5도/2mm/유지 기준을 통과했다. 해당 자세에 직접 초기화한 끝점 검사이며 첫 파지에서의 도달 성공이 아니다. 압력 지령이 실물 전류나 토크 환산값도 아니다.
- 앞서 붙이던seed0은750ms 선별부터 통과한 압력 조합이 없었다. 원래 힘을 유지한 채 경로만 따라가는 문제 외에 목표 끝 자세 자체의 지지력도 확인할 필요가 있었다. 기존 실패 기록을 보존하고 검증된seed1의 끝 자세로 경로를 다시 만들었다.
- 검지 접촉을 끝까지 고정하는 대신 검지 또는 약지 접촉의 기하적 전환을 허용하고 엄지도 포함해 탐색했다. 이것은 탐색 비용/접촉 모드 변경이며 실제 관절·충돌·추종·2틱/75ms/1ms 기준의 완화가 아니다.
- 현재 정수 관측 첫 회전 끝 상태에서 새 경로는{bridge['goals_completed']}목표를 전부 진행하고 원래 동적 기준을 통과했다. 큐브를 놓치지는 않았지만 최종90도 자세 오차{bridge['endpoint']['target90_error_deg']:.3f}도, 마지막 약지 지지 비율{bridge['third_support_fraction']:.1%}로 전환 성공은 아니다. 세부 상태는 `rrt_rf_seed5_integer_equilibrium3/`에 있다.

## 끝 보정

위 실제 경로 끝 상태의qpos/qvel/ctrl/act/warmstart 차이0을 확인하고 보정을 연결했다. 자세를 원래 목표로 맞추는 보정은 방향 오차를 약15도에서8도까지 줄였지만 지지력을 확보하지 못했다. 손끝 거리와 실제 법선력을 구분한다. 작은 움직임의 독립 진단에서 검지–중지 중간 링크 간섭도 확인했고, 해당 분기를 거부한 채 가능한 동시 관절 움직임을 비교한다.

끝 자세 보정 중 경로 진행 번호가 더 증가하지 않는다는 이유로 중단하던 탐색 로직을 고쳤다. 원래 동적 기준은 그대로다. 의도적으로 탐색을 바꾸며 중단한v4/v5는 `offline_search_stop_requested`로 보존했고 실패한 실물 실행으로 세지 않는다. 새 변형은 실제 현재 자세에서 약지·엄지 하중을 먼저 확보한 다음 원래 목표로 서서히 복귀한다. 기하학적 계획 분기는 후보를 제안할 뿐 실제 큐브 상태를 고정하지 않는다.

{chr(10).join(lines)}

최종 보고서가 아직 없는 계산: {', '.join(pending) if pending else '없음'}. 최종 판정은 각`review.json`과 `summary.json`을 따른다. 초기화한 끝 자세의 유지, 실제 전환 경로, 다음90도와 최종 윗단 공간은 서로 다른 검증이다. 약지 임시 지지 자세에 윗단 공간이 확보됐다고 주장하지 않는다.

[준비 상태](summary.json) · [접촉력 검산](support_audit.json) · [끝 자세 검사](endpoint_screen/review.json) · [작은 움직임 진단](local_move_diagnostics.json) · [기존 첫 회전·유지](../STATUS.md)
'''
    (FOLDER/'STATUS.md').write_text(status,encoding='utf-8')
    rows=''.join(f'<tr><td><a href="../../{html.escape(r["path"])}">{html.escape(r["path"].split("/")[-2])}</a></td><td>{r["steps"]}</td><td>{"통과" if r["passed"] else "미달"}</td><td>{html.escape(str(r["stop"]))}</td></tr>' for r in branches)
    repair_rows=''.join(f'<tr><td><a href="../../{html.escape(r["path"])}">{html.escape(r["path"].split("/")[-2])}</a></td><td>{r["steps"]}</td><td>{"통과" if r["passed"] else "미달"}</td><td>{r["final"]["error_deg"]:.3f}도</td></tr>' for r in repair_trials)
    page=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>약지 하중 인계 후속</title><style>body{{font:16px/1.7 system-ui;max-width:1100px;margin:25px auto;padding:0 18px;color:#203341}}.note{{padding:16px;background:#fff0c9}}table{{border-collapse:collapse;width:100%}}td,th{{padding:8px;text-align:left;border-bottom:1px solid #ccc}}img{{max-width:100%}}</style>
<h1>약지 하중 인계 후속</h1><p class="note">전체 동작의 실물 준비 완료가 아니다. 접촉 계산10회에서 누락되던 얕은 접촉을50·100·200회에서 확인했다.50회 모델의 기존 정책은4118목표에서 중단됐고, 추가 SAC3만 스텝의 중간·최종 정책4개도 첫 회전·유지 미달이다. 이전 첫 회전 통과 결과는10회 모델의 기록이다. 모든 결과는 모델이며 장치를 열지 않았다.</p>
<p>추가 SAC의 완료 요청 분량 합계 {report['new_SAC_environment_steps_this_refinement']:,}스텝. 학습 창 점수와 아래 독립 초기 배치 평가를 구분한다. 원본 정책과 실패 결과를 보존하고 자동 교체하지 않는다.</p>
<p>현재 정책의 제안 보정 범위는 기준 궤적 주변±0.01rad, 약±6.519틱이다. 실행 간격 최대2틱/75ms와는 다른 값이다. 세 번의 추가 학습과 네 정책 재시험을 통해 아직 이 구조로 회전·유지를 통과하지 못했다. <a href="POLICY_REUSE_DECISION.md">재사용 판정과 남은 준비</a>.</p>
<img src="../../review/ccd_policy_comparison.png" alt="접촉 반복 횟수와 보정 정책별 초기 배치부터의 모델 비교"><p>점선은 과거10회 모델의 통과 기록이다. 그림은 저장 제어 주기 표본이며 실물 측정이나 모든1ms 상태의 인증이 아니다.</p>
<table><thead><tr><th>새 초기 배치 평가</th><th>목표 수</th><th>첫 회전·유지</th><th>마지막90도 오차</th></tr></thead><tbody>{repair_rows}</tbody></table>
<p>검증된 끝 자세 {endpoint_passes}개를 확보했다. 직접 초기화한 뒤 지탱하는 검사이며 도달 성공이 아니다. 현재 첫 회전 끝에서 새 경로1265목표는 큐브를 놓치지 않고 진행했지만 방향 오차15.94도와 약지 지지력 부족으로 미달이다.</p>
<p>실제 끝 상태를 이어 하중 인계 보정을 진행한다. 현재 자세에서 힘을 먼저 확보한 뒤 원래 방향으로 복귀하는 제어도 비교한다. 마지막 보고서 대기: {html.escape(', '.join(pending) if pending else '없음')}.</p>
<table><thead><tr><th>모델 제어 비교</th><th>목표 수</th><th>임시 약지 지지</th><th>종료 사유</th></tr></thead><tbody>{rows}</tbody></table>
<p><a href="STATUS.md">상세 기록</a> · <a href="summary.json">출처와 판정</a> · <a href="../../review/index.html">전체 검토 페이지</a></p></html>'''
    if sensitivity.exists():
        page=page.replace('<h1>약지 하중 인계 후속</h1>',
            '<h1>약지 하중 인계 후속</h1><p class="note"><a href="small_joint_response/STATUS.md">추가 확인: 관절 약0.6도 자극의 큐브 변화와 이미 저장한 약지4개 값</a>. 세 모델 상태의96조건 비교이며 실물 평균 오차가 아니다. 보정폭만을 전체 실패의 주원인으로 단정하지 않는다.</p>')
    (FOLDER/'index.html').write_text(page,encoding='utf-8')
    summary=f'''2026-11-10 가장 최근 판정: **접촉 계산10회 모델의 성공을 수렴 비교한50회 모델의 성공으로 이전할 수 없다.** 얕은 두 손끝 접촉이10회에서 누락됐지만50·100·200회에서 잡혔고, 같은4118명령의50회/100회 최종 상태 차이는0이었다. 원본 형상·관절·힘 한도·PID·마찰·중단 기준을 보존하고 수치 반복 상한만10→50으로 바꾼 별도 모델에서 기존 정책은 첫 회전4118목표 후 큐브80mm 이탈로 중단됐다. 원본170개와 과거6956목표/41초 유지 결과는 유지하지만 그 결과는10회 모델의 기록이다.

실제50회 모델 상태를 써 기존 SAC actor를 복원한 추가 SAC3회·총{report['new_SAC_environment_steps_this_refinement']:,}스텝을 완료했다. 실험별 실제 trainer 최종 번호도 각각10000이다. 짧은 창 중간/최종 정책은4267/4243목표, 실패 이후까지 포함한1200주기 창 정책은4313목표, 초반600주기부터 개입한 정책은4233목표에서 큐브를 놓쳤다. 네 정책 모두 실물 실행용으로 채택하지 않았다. 학습 창 초기화와 점수 개선은 전체 연속 성공이 아니다. 이전 완료 요청 분량60,000과 이번30,000을 합한 기록은90,000이다. 새 초기 배치 검증은 `summary.json`의 `SAC_CCD50_followups`, `fresh_repair_trials`를 따른다.

현재 잔차 제안은 기준 궤적 주변±0.01rad, 약±6.519틱이고 실행 간격 최대2틱/75ms와 구분한다. 초기화해 지지된 약지 후보 하나는 기존 첫 파지와 네 약지 지령이 약671/−518/−1127/585틱 달라 작은 잔차만으로 그 큰 재파지를 직접 구현하는 구조는 아니다. 그 후보가 최소 필수 이동량이라고 주장하지 않는다. 수렴 조건의 기준 동작/접촉 모드와 보정 범위를 함께 검토해야 하며 실패한 동작을 증류하지 않는다. 상세 판정은 `POLICY_REUSE_DECISION.md`다.

전환 구간에서는 모델 접촉력4상태를 검산하고 약지·엄지·검지 뿌리로 지탱하는 초기화 끝 자세{endpoint_passes}개를 찾았다. 첫 회전에서 도달한 결과가 아니다. 기존10회 모델의 초기 배치부터9049목표를 중간 초기화 없이 재현했으나 마지막 방향 오차14.066도와 지지 비율0.0988%로 작업 실패다. 뒤2093목표는 저장 모델 탐색 명령으로 전체 SAC가 아니다.

현재 정수 관측 첫 회전 끝에서 엄지 포함 새 경로1265목표는 큐브를 놓치지 않고 전부 진행했지만 방향 오차15.94도와 약지 지지력 부족으로 미달이다. 보정 중 방향 오차를 약8도까지 줄였으나 지속 하중 유지48비교와 중력 보상 등은 채택하지 못했다. **첫 회전의 수렴 모델 재검증·다음90도·전체 연결·실물 준비는 완료가 아니다.** 실물·원격·UI 변경은 없다. 사용자 연결·촬영·실험은 지금 필요하지 않다.

최신 근거: `right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/STATUS.md`, `summary.json`, `index.html`. 이전 하부 파지 사진 검토와 큐브 제거 확인을 되돌리지 않는다.'''
    insert(OUT/'WORK_STATUS.md','# 약지 하중 인계 후속\n\n'+summary)
    insert(ROOT/'cloud_handoff_20261106/CLOUD_START_PROMPT.md',summary,after='## 지금 먼저 할 일')
    insert(ROOT/'professor_progress_20261015.md','# 현재: 약지 하중 인계의 모델 검증\n\n'+summary)
    insert(ROOT/'right_hand_rotation_20261109/policy_audit/STATUS.md','# 최신: 약지 하중 인계 후속\n\n'+summary)
    print(json.dumps(dict(endpoint_holds=endpoint_passes,feedback_branches=len(branches),pending=pending,
                          third_support_motion_passed=report['third_support_motion_passed'],devices_opened=0)),flush=True)


if __name__=='__main__':main()
