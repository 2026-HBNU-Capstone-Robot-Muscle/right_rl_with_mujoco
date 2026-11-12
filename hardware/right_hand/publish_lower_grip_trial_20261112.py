"""Publish the verified, limited physical-observation entry and continuation."""
import json
from pathlib import Path
from lower_grip_trial_plan_20261112 import ROOT,OUT,CANDIDATE,REQUEST,PROOF,LowerGripTrialPlan,sha,candidate
from lower_grip_trial_gate_20261112 import validate_review

def put(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

def main():
    proof=validate_review(LowerGripTrialPlan(REQUEST),PROOF)
    software=json.loads((OUT/'software_verification.json').read_bytes());guards=json.loads((OUT/'runtime_guard_review.json').read_bytes())
    assert software['passed'] and software['tests']==9 and software['failures']==software['errors']==0
    assert guards['passed'] and guards['candidate_sha256']==sha(CANDIDATE)
    assert guards['checked_plan_sha256']==sha(ROOT/'lower_grip_trial_plan_20261112.py')
    assert guards['checker_sha256']==sha(ROOT/'screen_lower_grip_runtime_guards_20261112.py')
    assert proof['accepted_for_operator_trial'] and len(candidate()['goals'])==3194
    stress=ROOT/'right_hand_rotation_20261112/support_first_transfer/sequential_low_mu_v1/review.json'
    stress_row=json.loads(stress.read_bytes())
    ready=dict(operator_observation_ready=True,entry='leap_cycle_lower_grip.cmd',
        scope='First rotation -> upper-middle withdrawal before thumb transfer -> ring support -> three-finger support observation',
        loaded_goals=3194,candidate_sha256=sha(CANDIDATE),review_sha256=sha(PROOF),
        loaded_finite_cases=6,hand_only_cases=3,software_tests=9,runtime_held_progress_guard_replay_passed=True,
        inherited_empty_static_cases=201,inherited_empty_dynamic_cases=9,
        fresh_actual_OFF_recovery_must_be_screened_at_runtime=True,
        current_ma=450,maximum_goal_step_ticks=2,period_s=.075,approved_maximum_on_s=500,
        friction_hypotheses=[.2,.3],material_friction_measured=False,
        lower_friction_stress=dict(mu=.1,stop=stress_row['stop'],goal=stress_row['done'],path=stress.relative_to(ROOT).as_posix(),sha256=sha(stress)),
        upper_layer_rotation_space_ready=False,next_whole_cube_rotation_included=False,full_cycle_ready=False,
        real_cube_drop_auto_detection_available=False,physical_success=False,new_SAC_training_steps=0,
        assistant_opened_devices=0,assistant_motor_writes=0)
    put(OUT/'READY.json',ready)
    guide='''# 수정 파지 실물 관찰 시험\n\n실행: `leap_cycle_lower_grip.cmd`\n\n이전 실행파일은 보존했다. 이번 파일은 첫 회전 후 중지 윗마디를 먼저 펴고, 엄지로 이송한 뒤 검지·중지 아래받침과 약지로 지지하는 수정 경로를 관찰한다.\n\n## 실행 순서\n\n1. 빈손 상태에서 실행하고 첫 SPACE로 현재 OFF 자세 검사와 빈손 준비를 시작한다. 현재 자세가 저장 자세와 다르면 실행기가 새로 검사하며, 검사 거부 시 목표를 쓰지 않는다.\n2. PLACE 안내가 나오면 기존과 같이 **초록 윗면·주황 손목 ID41 방향**으로 큐브를 놓는다. 손을 뗀 뒤 두 번째 SPACE를 누른다.\n3. 첫 회전 → 기존 유지 → 중지 윗마디 먼저 펴기 → 엄지 이송 → 아래받침 이동 → 약지 지지 → 4.05초 유지가 자동 재생된다. 도중에 큐브가 미끄러지거나 떨어지면 Q/Esc/Ctrl+C로 중단한다. 모터 추종 기준만으로 큐브 낙하를 자동 판정하지 않는다.\n4. 관찰 종료 안내가 나오면 큐브 아래를 받친 뒤 마지막 SPACE로 토크 OFF하고 종료 후 큐브를 제거한다. 실행 중 자동 OPEN/OFF는 없다.\n\n사진 `cube_before_grasp.jpg`, `cube_grasp_hold.jpg`, `cube_lower_grip_hold.jpg`, 영상과 관절·전류·전압 기록은 새 `lower_grip_trial_.../` 폴더에 저장된다. 사용자가 별도로 로그나 사진을 다시 촬영할 필요는 없다.\n\n## 검사 범위\n\n- 마찰 0.2/0.3 가정 × 모델 구동 응답 0.5/1/1.5의 6조건에서 처음 배치부터 상태 초기화 없이 3194목표 전체를 검사했다. 모델 지지·추종·관절 제한·손 자체 및 받침 간섭 기준을 통과했고, 마지막 4050표본에서 엄지·중지·약지 접촉이 모두 성립했다.\n- 빈손 구동 응답 3조건, 소프트웨어 9시험과 실제 실행기의 고정 관절/진행 감시 재생을 통과했다. 원본 170개 결합 소스 해시는 소프트웨어 시험에서 확인했다.\n- 동일한 저장 OFF 씨드의 빈손 준비 경로가 이전 검증과 완전히 같음을 확인해 정적 201/동적 9조건 기록을 상속했다. 실물 실행 시 현재 OFF 자세를 다시 읽고 필요한 빈손 검사를 수행한다.\n- 450mA, 기존 PID, 최대 2틱/75ms, 기존 건강·전류·전압·통신·추종·진행·고정 관절 중단 기준 및 승인된 최대 500초를 유지했다. 500을 다시 입력할 필요는 없다.\n\n## 아직 인증되지 않은 부분\n\n**이번 파일에는 다음 회전과 왼손 동작이 없다.** 윗단 회전 공간 검사는 미통과 상태로 보존했으며, 모델에서는 큐브 윗면도 바뀌고 기울기가 남았다. 따라서 이것은 수정된 세 손가락 지지의 실물 관찰 시험이고, `회전 → 유지 → 다음 회전 → 유지` 전체 동작 준비 완료를 뜻하지 않는다. 윗단 공간 합격 조건을 낮춰 통과 처리하지 않았다. 실제 왼손이나 윗단을 움직이는 명령도 포함하지 않았다.\n\n플라스틱의 실제 마찰계수는 측정하지 않았다. 0.2/0.3은 설계 비교 가정이며, 더 낮은 0.1 가정은 첫 경로의 목표11에서 큐브가 외부 받침에 닿아 거부됐다. 재료 이름만으로 모든 마찰 조건이 검증됐다고 간주하지 않는다.\n\n모델 접촉 보정으로 만든 명령을 저장해 재생한다. 실물에서 힘 센서를 읽거나 SAC가 실시간으로 보정하는 실행은 아니다.\n'''
    (OUT/'operator_guide.md').write_text(guide,encoding='utf-8')
    marker='lower_grip_operator_observation_ready_20261112'
    note=f'''\n<!-- {marker} -->\n## 최신: 저마찰 가정을 반영한 수정 파지 실물 관찰 파일 준비\n\n사용자가 경로 수정과 실물 테스트 준비를 요청했고 `leap_cycle_lower_grip.cmd`를 새로 준비했다. 기존 실물 실행파일은 보존했다. 첫 회전 후 중지 윗마디를 먼저 펴고 엄지로 이송한 뒤, 아래받침 이동과 약지 지지를 거쳐 엄지·중지·약지 지지를 관찰한다.\n\n후보3194목표를 마찰0.2/0.3 × 모델 구동응답0.5/1/1.5의 6조건에서 초기화 없이 검사했고 지지 및 모든 기존 구동·관절·간섭 중단 기준을 통과했다. 마지막4050표본에서 세 손가락 접촉을 확인했다. 빈손3조건, 소프트웨어9시험, 고정 관절/진행 감시 재생도 통과했다. 실제 성공으로 표현하지 않는다. 450mA/PID/2틱75ms 및 기존 중단 기준과 승인500초를 유지하며 추가 승인 입력은 없다. 실물 시작은 사용자 .cmd와 SPACE다. 보조자는 장치를 열거나 모터를 구동하지 않았다.\n\n**이번 파일은 수정 파지 관찰 시험이며 다음 회전·왼손 동작은 미포함이다.** 윗단 공간 미통과, 큐브 작업면 변화와 기울기를 실패 기록으로 보존했다. 전체 연속동작 준비 완료로 안내하지 않는다. 마찰0.1 가정은 초기 목표11에서 외부 받침 접촉으로 거부됐고, 플라스틱의 실제 마찰계수는 측정하지 않았다.\n\n실행 안내와 근거: `right_hand_rotation_20261112/lower_grip_trial/operator_guide.md`, `READY.json`, `review.json`. 실행 결과는 `lower_grip_trial_.../`에 저장된다. 사용자 실행 후 접근 가능한 새 결과부터 확인하고, 현재 실물 파지 위치를 기준으로 윗단 공간과 다음 회전을 연결한다. 동일한 과거 사진 촬영·큐브 제거 확인을 다시 요구하지 않는다.\n<!-- /{marker} -->\n'''
    for path in (ROOT/'cloud_handoff_20261106/CLOUD_START_PROMPT.md',ROOT/'professor_progress_20261015.md'):
        doc=path.read_text(encoding='utf-8-sig')
        if marker in doc:
            begin=doc.index('<!-- '+marker+' -->');end=doc.index('<!-- /'+marker+' -->',begin)+len('<!-- /'+marker+' -->')
            doc=doc[:begin]+note.strip()+doc[end:]
        else:
            first,sep,rest=doc.partition('\n');doc=first+'\n'+note+'\n'+rest
        path.write_text(doc,encoding='utf-8')
    target_path=ROOT/'right_hand_rotation_20261112/support_first_transfer/FINAL_TARGET.json';target=json.loads(target_path.read_bytes())
    target.update(supervised_lower_grip_observation_ready=True,latest_operator_entry='leap_cycle_lower_grip.cmd',
        latest_operator_trial_ready_report='right_hand_rotation_20261112/lower_grip_trial/READY.json',
        full_rotation_hold_next_rotation_cycle_ready=False,current_physical_trial_is_only_intermediate_transfer_observation=False,
        current_physical_trial_is_lower_grip_observation=True)
    put(target_path,target)
    print(json.dumps(dict(operator_observation_ready=True,entry=ready['entry'],loaded_goals=3194,full_cycle_ready=False,devices_opened=0)))

if __name__=='__main__':main()
