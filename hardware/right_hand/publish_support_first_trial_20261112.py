"""Publish the tested operator packet; never open hardware or grant new scope."""
import json
from support_first_trial_plan_20261112 import ROOT,OUT,REQUEST,PROOF,CANDIDATE,SupportFirstTrialPlan,sha
from support_first_trial_gate_20261112 import validate_review

def put(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

def main():
    r=validate_review(SupportFirstTrialPlan(REQUEST),PROOF)
    tests=json.loads((OUT/'software_verification.json').read_bytes());runtime=json.loads((OUT/'runtime_guard_review.json').read_bytes())
    assert tests['passed'] and runtime['passed'] and runtime['candidate_sha256']==sha(CANDIDATE)
    assert runtime['checked_plan_sha256']==sha(ROOT/'support_first_trial_plan_20261112.py')
    assert runtime['checker_sha256']==sha(ROOT/'screen_support_first_runtime_guards_20261112.py')
    assert r['accepted_for_operator_trial'] and not r['physical_transfer_success']
    put(OUT/'READY.json',dict(status='prepared_for_user_started_support_first_transfer_trial',
        entry='leap_cycle_support_first_transfer.cmd',candidate_sha256=sha(CANDIDATE),proof_sha256=sha(PROOF),
        loaded_goals=2868,loaded_finite_conditions=6,hand_only_conditions=3,software_tests=tests['tests'],
        nominal_held_and_progress_policy_replay_passed=True,saved_seed_empty_static_cases=201,saved_seed_empty_dynamic_cases=9,
        saved_seed_empty_proof_inherited_by_exact_path_equality=True,fresh_actual_OFF_screen_required=True,
        current_ma=450,maximum_goal_step_ticks=2,sample_period_s=.075,maximum_on_seconds=500,
        previous500s_approval_retained=True,operator_SPACE_required=True,assistant_started_motion=False,
        physical_transfer_success=False,final_left_alignment_verified=False,next_rotation_included=False,
        all_condition_safety_certified=False,devices_opened=0))
    text='''# 받침 우선 파지 전환: 실물 관찰 시험

실행 파일: 루트의 **leap_cycle_support_first_transfer.cmd**. 창 제목에 **받침 우선 파지 시험**이 표시된다.

이번 범위는 기존 첫 회전과10초 유지 → 중지 아래받침 정렬 → 엄지로 중지 쪽 이동 → 위마디/엄지 작업 파지 →4.05초 유지다. 다음 회전과 왼손 동작은 포함하지 않는다. 중간90도 오차는 파지 전환의 합격 조건에서 분리했고 최종 왼손 파지 정렬은 아직 검증하지 않았다.

## 실행 순서

1. 큐브를 손 밖에 두고 새 `.cmd`를 실행한다. 첫 SPACE로 현재 OFF 자세를 읽고 새 시작 자세의 빈손 복귀를 검사·준비한다.
2. PLACE가 표시되면 **초록면 위 / 주황면 손목ID41 방향**으로 놓는다. 손을 뗀 다음 두 번째 SPACE로 첫 회전과 새 파지 전환을 시작한다. 화면에 받침 정렬, 엄지 이동, 위마디 이동, 유지 순서가 표시된다.
3. 끝의 관찰 종료 안내 뒤 큐브 아래를 받치고 마지막 SPACE로 OFF·설정 복구를 진행한다. 종료 확인 뒤 제거한다. 큐브가 떨어지거나 이상하면 기존 Q/Esc/Ctrl+C로 중단한다.

450mA·기존PID·최대2틱/75ms·관절 범위와 전류/전압/온도/추종/진행/고정관절/통신 중단 기준을 유지한다. 승인된 최대500초를 유지하므로500 입력을 다시 요구하지 않는다. 저장 시작 자세 기준 지령 약280.4초이며 배치 대기는 별도다. 자동OPEN이나 중간 토크OFF는 없다. 직접 장치 명령은 사용자의 실행과SPACE가 시작한다.

## 준비 확인과 범위

- 새 지령2868개 중 앞1417개는 이전 실물에서 실행한 첫 회전 경로와 같다.
- 마찰0.2/0.3·구동배율0.5/1/1.5의6조건과 큐브 없는 손3조건을 처음 배치부터 매1ms 검사했다.9조건 모두 통과했다.6조건의 최종 지지와 윗단 공간은 각각4050/4050개이고 최소 윗단 여유는 약6.381mm다. 마찰/구동배율/힘은 실물 보정값이 아니다.
- 기존 고정관절20틱 구간과 진행 감시를 명목 모델 관측에 적용한 추가 검사, 소프트웨어9검사를 통과했다. 기존 원본170개 해시를 확인했다.
- 저장된 시작 자세의 빈손201정적/9동적 검사는 이전 경로와 지령·시작이 정확히 같아 계승했다. 현재 실물 자세는 실행 때 다시 읽으며 다르면 새로 검사한다.
- 실물 낙하 방지·안정 파지·실제 윗단 공간, 배치 오차, 다른 손의 하중과 최종 왼손 정렬은 아직 확인 전이다. 프로그램 완료를 실제 파지 성공으로 자동 판정하지 않는다. 저장 경로 재생이며 새SAC 학습이나 증류가 아니다.

기록은 `support_first_trial_날짜_시간/`에 저장된다. 전체 영상과 각 관절 목표/실제값, 마지막 `cube_support_first_hold.jpg`를 함께 검토한다. 기존 `leap_cycle_nominal_two_turns.cmd`의 낙하한v3과 이번 파일은 서로 다른 경로다.

근거: `READY.json`, `review.json`, `models/`, `runtime_guard_review.json`, `software_verification.json`, `empty_prefix_inheritance.json`. 새 실물 결과는 아직 없다.
'''
    (OUT/'operator_guide.md').write_text(text,encoding='utf-8')
    put(OUT/'delivery.json',dict(entry='leap_cycle_support_first_transfer.cmd',ready_sha256=sha(OUT/'READY.json'),
        guide_sha256=sha(OUT/'operator_guide.md'),proof_sha256=sha(PROOF),
        operator_SPACE_required=True,assistant_started_motion=False,new_scope_approval_requested=False,
        actual_physical_success=False,devices_opened=0,remote_writes=0))
    print('Support-first trial ready for operator SPACE; no device opened.')

if __name__=='__main__':main()
