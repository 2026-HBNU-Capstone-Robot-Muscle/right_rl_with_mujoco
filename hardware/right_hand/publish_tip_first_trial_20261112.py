"""Publish the verified, limited distal-first physical observation trial."""
import json
from pathlib import Path
from tip_first_trial_plan_20261112 import ROOT,OUT,CANDIDATE,REQUEST,PROOF,TipFirstTrialPlan,sha,candidate
from tip_first_trial_gate_20261112 import validate_review

def put(path,value):path.write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')

def main():
    proof=validate_review(TipFirstTrialPlan(REQUEST),PROOF)
    software=json.loads((OUT/'software_verification.json').read_bytes());guards=json.loads((OUT/'runtime_guard_review.json').read_bytes())
    assert software['passed'] and software['tests']==9 and guards['passed']
    assert guards['candidate_sha256']==sha(CANDIDATE) and guards['checked_plan_sha256']==sha(ROOT/'tip_first_trial_plan_20261112.py')
    assert guards['checker_sha256']==sha(ROOT/'screen_tip_first_runtime_guards_20261112.py')
    z=candidate();assert len(z['goals'])==3110
    ready=dict(operator_observation_ready=True,entry='leap_cycle_tip_first_transfer.cmd',candidate_sha256=sha(CANDIDATE),review_sha256=sha(PROOF),loaded_goals=3110,
        user_confirmed_rearward_direction='orange side farther from white/front edge',rearward_distance_in_model_mm=3.,
        rearward_distance_physically_registered=False,tip_ids=[1,9],next_link_ids=[2,11],
        native_limit_fraction_proposal=.5,commanded_maximum_flexion_not_certified=True,model_lookahead_ticks=40,
        nonprimary_loaded_goal_relief=True,loaded_model_cases=6,hand_only_cases=3,software_tests=9,pure_proposal_tests=4,
        runtime_held_progress_guards_passed=True,all_native_stop_thresholds_unchanged=True,
        current_ma=450,goal_step_ticks=2,period_s=.075,approved_max_on_s=500,
        ring_closing_included=False,three_finger_final_grasp_included=False,next_rotation_included=False,full_cycle_ready=False,
        real_cube_drop_automatically_detected=False,actual_run_performed=False,assistant_motor_writes=0,devices_opened=0)
    put(OUT/'READY.json',ready)
    guide='''# 뒤쪽 접촉·끝마디 우선 이동 시험\n\n실행: `leap_cycle_tip_first_transfer.cmd`\n\n사용자가 확인한 뒤쪽은 주황 옆면에서 흰 앞면 모서리와 멀어지는 방향이다. 모델에서는3mm 접촉 이동을 가정했다. 실물3mm를 측정하거나 보증한 값은 아니다.\n\n## 달라진 동작\n\n기존 첫 회전 → 중지 윗마디 먼저 펴기 → 엄지 이송 → 뒤쪽 접촉 목표 → 엄지 ID1·중지 ID9 끝마디 먼저 굽히기 → 엄지 ID2·중지 ID11 굽히기 →4.05초 관찰. 끝마디 굽힘에 의해 눌리는 다른 마디의 목표는 조금 풀어 주도록 모델에서 경로를 계산했다. 이 기록을 재생하며 실물에서 모델 힘 센서나 실시간 SAC를 사용하는 것은 아니다.\n\n최대 굽힘을 곧바로 강제하지 않는다. 원래 범위 안의 절반 방향 후보에 목표 압박 제한을 적용했다. 실제 최대 관절값까지 도달했다고 인증하지 않았다.\n\n## 실행\n\n1. 빈손으로 새파일을 실행하고 첫 SPACE로 현재 OFF 자세 검사·준비를 시작한다.\n2. PLACE 안내 때 **초록 위·주황 손목 ID41 방향**으로 큐브를 놓고 손을 뗀 뒤 두 번째 SPACE를 누른다.\n3. 큐브가 미끄러지거나 떨어지면 Q/Esc/Ctrl+C로 중단한다. 모터 감시만으로 큐브 낙하를 자동 판정하지 않는다.\n4. 관찰 종료 안내 때 큐브 아래를 받친 뒤 마지막 SPACE로 토크 OFF하고, 종료 후 큐브를 제거한다.\n\n사진 `cube_tip_first_hold.jpg`, 기존 배치·파지 사진, 영상과 모터 기록은 새 `tip_first_trial_.../`에 자동 저장된다. 따로 촬영하거나 로그 전체를 붙여 넣을 필요는 없다.\n\n## 검증 범위\n\n마찰0.2/0.3 가정 × 모델 구동 응답0.5/1/1.5의6조건에서 처음부터 초기화 없이3110목표를 검사해 기존 구동·간섭·지지 기준을 통과했다. 빈손3조건, 실행기9시험, 경로 제안 모듈4시험과 고정 관절·진행 감시 재생을 통과했다. 동일한 저장 씨드의 빈손201정적/9동적 검증을 상속했고 현재 OFF 자세는 실행 시 다시 검사한다. 원본170개 결합 해시는 실행기 시험에서 확인했다.\n\n450mA·기존 PID·최대2틱/75ms·기존 건강/전압/전류/통신/추종/진행/고정 관절 중단 기준과 승인된 최대500초를 유지한다. 추가 승인값 입력은 없다. 이전64틱 목표 여유 후보는 진행 감시에서 거부돼 보존했으며, 이번40틱 목표 여유는 중단 기준 변경이 아닌 경로 자체의 압박 감소다.\n\n**이번 파일은 제안한 밀기 순서를 확인하는 관찰 시험이다. 지난번 ID15에서 중단한 약지 닫힘, 최종 세 손가락 파지, 다음 회전과 왼손 동작은 포함하지 않는다.** 모델의 큐브 기울기도 남으므로 전체 작업 완료나 왼손 인계 준비 완료로 판정하지 않는다. 실제 저마찰 재료의 계수는 측정하지 않았다.\n'''
    (OUT/'operator_guide.md').write_text(guide,encoding='utf-8')
    marker='tip_first_rearward_operator_trial_ready_20261112'
    note=f'''\n<!-- {marker} -->\n## 최신: 뒤쪽 접촉·끝마디 우선 밀기 관찰 파일 준비\n\n사용자는 `lower_grip_trial_20261112_162806_610535`를 실행했고 엄지 접촉을 주황 옆면의 흰 앞면에서 더 먼 뒤쪽으로 옮기며, 엄지·중지 끝마디를 먼저 굽힌 뒤 다음 마디로 밀자고 요청했다. 저장 영상의 해당 단계 프레임을 실제 확인했다. 이 실물 실행은 약지 ID15 목표3076/실제2957틱,281mA,0.183rad 추종 기준으로 중단했다. 토크 OFF와 설정 복구는 확인됐으며 추가 제거 확인을 요구하지 않는다. 근거는 `right_hand_rotation_20261112/lower_grip_actual_review/`다.\n\n새 실행파일은 `leap_cycle_tip_first_transfer.cmd`, 준비 근거는 `right_hand_rotation_20261112/tip_first_trial_v2/READY.json`과 `operator_guide.md`다. 기존 첫 회전과 중지 빼기/엄지 이송을 유지하고, 뒤쪽3mm 모델 접촉 가정 후 엄지1·중지9 끝마디 → 엄지2·중지11 다음 마디 순서로 굽힌다. 눌리는 비주동 마디의 목표를 조금 풀며 원래 한계의 절반 방향 후보를 사용한다. 최대 굽힘까지 달성했다고 표현하지 않는다. 기존64틱 목표 여유 후보는 진행 감시에서 거부됐고 새40틱 목표 여유는 동일한 중단 기준을 통과했다.\n\n마찰0.2/0.3 × 모델 구동응답0.5/1/1.5의6조건과 빈손3조건에서3110목표 전체를 초기화 없이 검사했다. 실행기9시험, 순서 제안4시험과 고정 관절/진행 감시를 통과했다. 450mA/PID/2틱75ms, 모든 기존 중단 기준과 승인500초를 유지한다. 보조자는 장치를 열거나 모터를 구동하지 않았다.\n\n**이번 파일은 밀기 관찰 구간까지이며, 지난번 막힌 약지 닫힘·최종 세 손가락 파지·다음 회전·왼손은 미포함이다.** 실제 큐브 기울기와 다음 파지 위치를 새 저장 결과에서 확인한 뒤 연결한다. 이전 파일을 새 수정본으로 안내하지 않는다.\n\n사용자는 코드 변경을 GitHub에 수시로 반영해 달라는 팀 요청을 전달했다. 검증한 변경 묶음은 `feature/hardware`에 커밋·푸시하고 원격 커밋을 확인한다. 팀 원본 `right_rl_with_mujoco`에는 독립 검토 가능한 경로 모듈을, 개인 비공개 `xlerobot`에는 실행기와 검증 자료를 보존한다. `apps/monitoring_ui`는 수정하지 않는다. GitHub 앱 쓰기403은 기존 Git 인증으로 처리했다.\n<!-- /{marker} -->\n'''
    for path in (ROOT/'cloud_handoff_20261106/CLOUD_START_PROMPT.md',ROOT/'professor_progress_20261015.md'):
        doc=path.read_text(encoding='utf-8-sig')
        if marker not in doc:
            first,sep,rest=doc.partition('\n');path.write_text(first+'\n'+note+'\n'+rest,encoding='utf-8')
    target_path=ROOT/'right_hand_rotation_20261112/support_first_transfer/FINAL_TARGET.json';target=json.loads(target_path.read_bytes())
    target.update(latest_operator_entry=ready['entry'],latest_operator_trial_ready_report='right_hand_rotation_20261112/tip_first_trial_v2/READY.json',
        current_physical_trial_is_distal_first_transfer_observation=True,current_physical_trial_is_lower_grip_observation=False,
        current_physical_trial_is_only_intermediate_transfer_observation=True,latest_actual_lower_grip_trial_stopped=True,
        full_rotation_hold_next_rotation_cycle_ready=False)
    put(target_path,target)
    print(json.dumps(dict(operator_observation_ready=True,entry=ready['entry'],loaded_goals=3110,full_cycle_ready=False,devices_opened=0)))

if __name__=='__main__':main()
