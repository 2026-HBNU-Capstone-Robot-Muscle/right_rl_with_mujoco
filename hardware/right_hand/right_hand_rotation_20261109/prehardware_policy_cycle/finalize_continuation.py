"""Publish local, evidence-backed continuation state; no network or devices."""
from pathlib import Path
import json
import hashlib

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
BEGIN='<!-- prehardware_continuation_20261110 -->'
END='<!-- prehardware_continuation_20261110_end -->'


def load(path):return json.loads(path.read_text(encoding='utf-8'))
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def insert(path,text,after=None):
    source=path.read_text(encoding='utf-8')
    if BEGIN in source:
        start=source.index(BEGIN);end=source.index(END,start)+len(END)
        source=source[:start]+source[end:].lstrip('\n')
    block=BEGIN+'\n'+text.rstrip()+'\n'+END+'\n\n'
    if after:
        point=source.index(after)+len(after);source=source[:point]+'\n\n'+block+source[point:].lstrip('\n')
    else:source=block+source
    path.write_text(source,encoding='utf-8')


def main():
    sac_steps=sum(load(path).get('steps',load(path).get('timesteps',0)) for path in OUT.glob('sac_*/result.json'))
    connected=[]
    for case in ('nominal','x_plus05','friction025'):
        path=OUT/('connected_sac_student_hold_'+case)/'review.json';r=load(path)
        assert r['first_rotation_and_hold_passed'] and not r['suffix_is_saved_model_teacher_not_SAC']
        connected.append(dict(case=case,path=path.relative_to(OUT).as_posix(),sha256=digest(path),
            yaw_deg=r['endpoint']['yaw_deg'],orientation_error_deg=r['endpoint']['target90_error_deg'],
            station_error_mm=r['common_station_error_mm'],gap_mm=r['final_hold']['gap'],
            lower_samples=r['final_hold']['lower'],samples=r['final_hold']['samples'],
            goals=r['goals_completed'],passed=r['first_rotation_and_hold_passed']))
    packaged=[]
    for path in sorted(OUT.glob('next_cycle/staged_model_*/review.json')):
        r=load(path);packaged.append(dict(path=path.relative_to(OUT).as_posix(),sha256=digest(path),
            case=r['case'],integer_encoders=r['integer_encoder_observations'],model_kind=r.get('model_kind','active'),
            actor_override=r.get('correction_actor_research_override'),passed=r['first_rotation_and_hold_passed'],
            admissible_action_selection=r.get('native_range_admissible_correction_selection',False),
            admissible_selection_events=len(r.get('admissible_correction_events',[])),
            steps=r['steps'],stop=r['stop'],reasons=r['reasons'],final=r['final'],
            final_hold=r['final_hold']))
    bc=[]
    for path in sorted(OUT.glob('next_cycle/recovery_bc_*/training.json')):
        r=load(path);bc.append(dict(path=path.relative_to(OUT).as_posix(),sha256=digest(path),
            updates=r['updates'],data_examples=r['dataset_examples'],new_SAC_steps=r['new_SAC_training_steps'],
            encoder_augmentation=r.get('encoder_quantization_augmentation',False)))
    shadow_path=OUT/'next_cycle/staged_shadow_runtime_verification.json';shadow=load(shadow_path)
    assert shadow['status']=='passed' and shadow['model_fixture_goal_comparisons']==19668
    next_path=OUT/'next_model_feedback_90_from_student_hold/review.json';next_turn=load(next_path)
    assert not next_turn['success']
    converged=[r for r in packaged if r['model_kind']=='active_ccd50']
    readiness=dict(date='2026-11-10',connected_first_stage_model_cases=connected,
        connected_cases_also_used_for_teacher_data=True,packaged_controller_model_trials=packaged,
        behavior_cloning_runs=bc,new_SAC_environment_steps=sac_steps,
        SAC_step_count_is_completed_requested_budget_not_exact_trainer_counter=True,
        historical_first_stage_success_model_CCD_iterations=10,
        old_CCD10_successes_not_transferred_to_CCD50=True,
        current_converged_model=dict(kind='active_ccd50',CCD_iterations=50,
            first_rotation_and_hold_passed=any(r['passed'] for r in converged),
            trial_paths=[r['path'] for r in converged],
            whole_cycle_passed=False,physical_validation_completed=False),
        shadow_fixture_goal_comparisons=shadow['model_fixture_goal_comparisons'],
        shadow_verification_sha256=digest(shadow_path),
        first_stage_controller='reference + SAC residual; separate behavior-cloned final correction; native fixed-position servo hold',
        next_turn=dict(path=next_path.relative_to(OUT).as_posix(),sha256=digest(next_path),
            steps=next_turn['steps'],passed=False,final=next_turn['final']),
        blockers=['CCD50 first rotation and hold must be revalidated; retain the failed4118-goal parent trial',
                  'No validated load transfer to the ring finger and no second90-degree completion',
                  'Packaged controller sensor and unseen-condition validation is incomplete; retain all negative trials',
                  'Physical camera-to-policy registration and mounted geometry uncertainty are not validated',
                  'Model force/torque limits have no validated450mA conversion'],
        no_state_reset_in_connected_trials=True,physical_successes_this_work=0,
        repeated_confirmation_trials_are_not_independent_conditions=True,
        whole_cycle_distillation_completed=False,second_rotation_passed=False,
        hardware_execution_allowed=False,devices_opened=0,remote_repositories_changed=False,monitoring_ui_changed=False)
    (OUT/'next_cycle/readiness.json').write_text(json.dumps(readiness,ensure_ascii=False,indent=2),encoding='utf-8')
    table='\n'.join(f"| {r['case']} | {r['yaw_deg']:.3f}° | {r['orientation_error_deg']:.3f}° | {r['station_error_mm']:.3f}mm | {r['gap_mm']:.3f}mm | {r['lower_samples']}/{r['samples']} |" for r in connected)
    trial_lines='\n'.join(f"- [{r['path'].split('/')[-2]}](../{r['path']}): {'통과' if r['passed'] else '미달'}, {r['steps']}목표, 자세 오차{r['final']['error_deg']:.3f}도, 위치 오차{r['final']['center_error_mm']:.3f}mm. 범위 안 행동 선택{r['admissible_selection_events']}회. 중단: `{r['stop']}`." for r in packaged)
    detail=f'''# 실기 전 후속 준비: 2026-11-10

**첫 회전·유지는 접촉 계산10회 모델의 세 조건에서 학습 보정까지 연결했다. 수렴 비교한50회 모델의 성공으로 이전하지 않는다. 다음 회전과 실물 준비 완료는 아니다.** 최신 수치 진단과 추가 SAC 결과는 [접촉 계산·약지 인계 후속](handoff_refinement_20261110/STATUS.md)을 먼저 따른다. 물리 장치·원격 저장소·미들웨어 UI를 사용하거나 변경하지 않았다.

## 처음부터 연결한 첫 회전·유지

초기 배치→기준 궤적과 SAC 잔차6300목표→학습한 증분 보정→위치 지령 유지, 총6556목표를 세 조건에서 새로 실행했다. 손·큐브 상태 초기화는 없고 앞부분의 기존 SAC 경계 상태 차이는0이었다. 끝 보정에는 저장 교사 관절 목표를 재생하지 않았다.

| 모델 조건 | 회전 | 전체90도 자세 오차 | 공통 위치 오차 | 윗단 인증 하한 | 하부 접촉1ms 상태 |
|---|---:|---:|---:|---:|---:|
{table}

접촉·윗단 인증은 마지막4.05초의4050상태다. 양 손끝 법선력 각각0.2N 초과를 하부 접촉으로 센 모델 평가이며 실물 하중 측정이 아니다. 세 조건이 교사 학습 자료에도 포함돼 있으므로 독립적인 일반화 성공률이 아니다.6556목표의 모델 제어 시간은491.7초이며 속도를 최적화하지 않았다.

앞부분은 기준 궤적+SAC 잔차, 끝 보정은 교사 모방 학습, 이후는 원래 위치 서보다. 유지 중 토크를 끄지 않는다. 전체 순수 SAC나 전체 회전 정책 증류 완료로 부르지 않는다. 완료 SAC의 요청 학습 분량 합계는{sac_steps:,}스텝이며 실제 trainer의 묶음 진행 번호는 각 progress.json에 별도로 남긴다. 별도 모방 학습{len(bc)}회의 합계{sum(r['updates'] for r in bc):,}최적화 갱신을 여기에 더하지 않는다.

## 실제 입력 조건의 모델 시험

{trial_lines}

첫v2 부품은 소수 틱 모델 관측의 세 조건을 재현했으나 정수 인코더 관측에서는 끝 보정7주기 뒤 범위 밖 목표 제안으로 거부됐다. 실제 범위 밖 목표를 실행한 것이 아니다. 미학습Y+0.5mm도 끝 보정 중 범위 제안을 거부했고, 가정 마찰0.35는 앞 SAC 구간에서 큐브80mm 이탈로 중단됐다. 원래 결과를 보존했다.

긴 피드백 교사 세 궤적768표본은 재생 상태 오차0을 확인했다. 현재 각도와 최신 관절 오차 이력에 같은 반 틱 잡음을 주고 과거 이력에는 별도 잡음을 주는 정수 인코더 증강을 추가해v3 보정 부품을 별도 학습했다. 위 표의 결과가 실제 폐루프 판정이며 모방 정확도로 대체하지 않는다. 검증 표본을 학습에도 쓰므로 훈련 자료 재현 점수를 독립 성능으로 부르지 않는다.

증강 학습만으로 미달은 해결되지 않았다. 원래 범위 안에서 가장 가까운 증분 목표를 선택하는 별도 변형은 정수 인코더 명목 모델에서 초기 배치부터6956목표를 통과했다. v2 부품의 최종 자세 오차1.36867도, 공통 위치 오차1.49802mm, 마지막4050상태 하부 접촉100%, 원통 여유 인증 하한0.57195mm다. 위치 유지 전환 뒤 약41.55초를 이어갔으며 원래 관절/충돌/추종 기준을 통과했다. 범위를 늘리거나 범위 밖 목표를 실행하지 않았다. v3+범위 안 선택도 통과했지만 다른 정책 변형의 동일 명목 조건으로 구분한다. 준비 묶음의 `AdmissibleShadowController`가 이 변형이며 다음 회전 기능은 없다.

## 다음 회전이 막힌 구간

약지가 큐브에 닿는 기구학적 자세를 찾았다. 다른 손가락을 고정하면 간섭이 크지만 검지·중지를 함께 조정한 정적 자세4개와 검지 끝 접촉을 남기는 정적 자세3개는 형상 검사를 통과했다. 손가락 이동 여섯 순서의12개 비교와 RRT 우회 경로를 시험했다. 손가락 겹침 없는 경로도 실제 자유 동역학에서 큐브를 놓쳤다. 기하적 접촉 유지와 실제 지지력 유지가 다르다.

지지력 피드백·여러 관절 동시2틱 후보·150ms 앞 간섭·검지 뿌리 받침 고정도 비교했지만 약지 하중 인계를 끝내지 못했다. 새 학습 보정 뒤 파지에서 다음 절대180도 목표도{next_turn['steps']}목표 후 오차{next_turn['final']['error_deg']:.3f}도, 공통 위치 오차{next_turn['final']['common_station_error_mm']:.3f}mm로 정체됐다. 다음90도 성공은 없다. 모든 재파지의 불가능성을 입증한 결과는 아니다.

## 연결부와 보존한 원본

출력 API 없는 NumPy 단계 연결부를 만들고, 세 조건의19668목표가 저장 모델 평가와 전부 일치함을 확인했다. 오래된 표본·모델/순서가 다른 공간 근거·확인 응답 오류를 거부하며 확인 전에는 제어 단계를 바꾸지 않는다. 지원하지 않는 다음 회전 요청은 유지 지령을 바꾸지 않는다. 이 검사는 소수 틱 모델 관측이며 실물 카메라/인코더 정확도 검증이 아니다.

빠른 절단 단면/방사 지표는 원통 끝면 주변의 전체 최소거리 하한이 아니다. X+0.5mm의 빠른 지표0.141mm와 독립 분리면 인증0.313mm는 다른 계산이다. Y+0.5mm+범위 안 선택에서는 목표 각도·위치를 맞췄지만 독립 인증이 실패했다. 끝 상태 검지 끝의 독립 최근접점 거리 상한0.258743884mm와 정확한 지지 함수의 분리 하한0.258743884mm가 일치해0.3mm 여유 미달을 확인했다. 단면 지표를 큰 공간 여유 증명으로 쓰지 않는다. 연결부v2의 입력 명칭도 인증과 획득용 지표를 구분하도록 고쳤고 앞선v1 묶음은 보존했다. 형상과0.3mm 기준은 그대로다. 기존 SAC 단독 마찰0.25도 독립 인증이 실패했으므로 통과로 바꾸지 않았다.

native 거리0과 최근접점 불일치가 있던 초기 `workspace_rf/`, `workspace_mf/` 진단은 철회 표시를 하고 보존했다. 기존 독립 볼록 형상 검사로 `workspace_*_checked/`를 다시 계산했다. 실패한 긴 고정 유지 자료와 명목 위치2mm 초과 교사는 학습에 채택하지 않았다.

실물450mA/PID/관절 범위/2틱/75ms/전압·전류·추종 중지 기준과 원래170개 결합 원본을 유지했다. 수동 ID14 참고값으로 범위를 확대하지 않았다. 현재 손·카메라 등록과 힘 한도 환산은 미검증이다. 사용자의 재연결·재촬영·제거 확인을 요구하지 않는다.

[그림과 전체 비교](../review/index.html) · [준비 상태](readiness.json) · [단계 연결부 검사](staged_shadow_runtime_verification.json) · [기존 SAC와 교사 재생 기록](../WORK_STATUS.md)
'''
    (OUT/'next_cycle/STATUS.md').write_text(detail,encoding='utf-8')
    negative_count=sum(not r['passed'] for r in packaged)
    summary=f'''2026-11-10 후속: **초기 배치→SAC 잔차6300목표→별도 학습 보정→위치 지령 유지, 총6556목표를 명목·X+0.5mm·가정 마찰0.25의 모델 세 조건에서 통과했다.** 중간 손/큐브 상태 초기화와 끝 보정의 저장 교사 관절 목표 재생은 없다. 명목 전체90도 자세 오차1.121도, 공통 위치 오차1.004mm, 마지막4050상태 하부 접촉100%, 윗단 인증 하한0.431mm다. 세 조건은 교사 자료에도 포함돼 일반화 성공률이 아니다. 앞부분은 기준 궤적+SAC, 끝 보정은 모방 학습, 이후는 원래 위치 서보다. 토크 OFF는 실험 종료 처리이며 실행 중 유지 방식이 아니다.

정수 틱 관측과 미학습 조건을 별도로 시험했고 현재{len(packaged)}개 비교 중{negative_count}개는 미달이다. 첫v2의 정수 틱 관측은 보정7주기 뒤 범위 밖 목표를 거부했다. 미학습Y+0.5mm도 범위 제안이 거부됐고 가정 마찰0.35는 SAC 앞부분에서 큐브80mm 이탈로 중단됐다. 긴 교사768표본과 반 틱 인코더 증강으로v3 보정도 비교했다. 구체적인 성공/실패는 최신 `next_cycle/STATUS.md`와 `readiness.json`의 모델 시험 표를 따른다. 완료 SAC의 요청 학습 분량 합계는{sac_steps:,}스텝이며 별도 모방 학습{len(bc)}회·{sum(r['updates'] for r in bc):,}갱신을 더하지 않는다. 이전 첫 회전 성공은 접촉 계산10회 모델이며,50회 모델 재시험은4118목표에서 중단돼 추가 보정을 별도로 수행한다.

**원래 범위 안의 증분 목표를 선택하는 보정 변형은 정수 인코더 명목 모델의6956목표와 약41초 위치 유지까지 통과했다.** v2 부품 기준 최종 자세 오차1.369도, 공통 위치 오차1.498mm, 마지막4050상태 하부 접촉100%, 윗단 인증 하한0.572mm다. 인코더 증강만으로 해결된 결과가 아니며, 동일 명목 조건의 재확인을 독립 성공 조건으로 합산하지 않는다. Y+0.5mm는 각도·위치가 맞아도 끝 상태의 실제 모델 여유0.258744mm로0.3mm 기준이 미달했다. 빠른 절단 단면 지표를 전체 거리 인증으로 부르지 않도록 바로잡고 `staged_shadow_bundle_v2`에 두 입력의 차이를 명시했다.

약지의 정적 접촉 자세와 겹침 없는 우회 경로는 찾았지만 검지에서 약지로 하중을 넘기는 자유 동역학을 통과하지 못했다. 새 보정 뒤 파지의 다음180도 목표도144목표 후 오차86.263도로 정체됐다. **다음90도와 전체 회전→유지→다음 회전→유지 및 실물 시험 준비 완료가 아니다.** NumPy 단계 연결부19668목표 일치와 오류 처리 검사는 통과했지만 실물 등록·힘 한도·정수 관측 강건성을 대신하지 않는다. 손/큐브를 고정하거나 범위·충돌 형상을 바꿔 완료로 만들지 않았다.

실물 장치를 열지 않았고 원래170개 실행 결합 원본과450mA/PID/범위/2틱/75ms/중지 기준을 유지한다. 원격 저장소와 `apps/monitoring_ui`는 변경하지 않았다. 사용자의 재연결·촬영·큐브 제거 확인은 지금 필요하지 않다. 최신 근거는 `right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/STATUS.md`, `next_cycle/readiness.json`, `review/index.html`이다. 아래2026-11-09의 저장 교사109목표 기록은 앞선 단계의 기록으로 남긴다.'''
    insert(OUT/'WORK_STATUS.md','# 2026-11-10 후속 상태\n\n'+summary)
    insert(ROOT/'cloud_handoff_20261106/CLOUD_START_PROMPT.md',summary,after='## 지금 먼저 할 일')
    path=ROOT/'cloud_handoff_20261106/CLOUD_START_PROMPT.md'
    source=path.read_text(encoding='utf-8').replace('아래는 2026-11-09까지 확인한 로컬 작업의 인계문이다.','아래는 2026-11-10까지 확인한 로컬 작업의 인계문이다.')
    path.write_text(source,encoding='utf-8')
    insert(ROOT/'professor_progress_20261015.md','# 현재: 실기 전 단계 제어와 정수 관측 검증\n\n'+summary)
    insert(ROOT/'right_hand_rotation_20261109/policy_audit/STATUS.md','# 최신: 학습 보정 연결과 남은 검증\n\n'+summary)
    banner='<p class="note"><strong>2026-11-10 후속 결과:</strong> 이전 첫 회전·유지 통과는 접촉 계산10회 모델의 기록이다. 누락 접촉을 확인해50회 별도 모델로 첫 회전부터 재검증했고 기존 정책은4118목표에서 중단됐다. 같은 명령의50/100회 최종 상태 차이는0이다. 기존 SAC의 보정 학습과 새 초기 배치 검증은 <a href="../../prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/index.html">최신 수렴·정책 검증</a>을 따른다. 전체 회전→유지→다음 회전 및 실물 준비 완료가 아니다. <a href="../../prehardware_policy_cycle/review/index.html">그림과 전체 검증 결과</a>. 아래는 이전 진단의 설명이다.</p>'
    for path in (ROOT/'render_policy_failure_explanation_20261109.py',ROOT/'right_hand_rotation_20261109/policy_audit/explanation/index.html'):
        source=path.read_text(encoding='utf-8');lines=source.splitlines()
        indices=[i for i,line in enumerate(lines) if line.startswith('<p class="note"><strong>2026-11-') and '후속 결과:' in line]
        if len(indices)!=1:raise RuntimeError('Expected exactly one dated continuation banner')
        lines[indices[0]]=banner;path.write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps(dict(local_documents_updated=True,packaged_model_trials=len(packaged),negative_trials=negative_count,devices_opened=0)),flush=True)


if __name__=='__main__':main()
