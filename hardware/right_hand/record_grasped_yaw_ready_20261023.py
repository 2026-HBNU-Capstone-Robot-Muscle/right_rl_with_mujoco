"""Record the retention report and verified next small-turn observation."""
import hashlib
import json
from pathlib import Path
from grasped_yaw_plan import GraspedYawPlan,CANDIDATE,CANDIDATE_SHA
from thumb_live_start_gate import validate_proof

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261023'


def main():
    validation_path=OUT/'grasped_yaw_validation.json';validation=json.loads(validation_path.read_bytes())
    integration=json.loads((OUT/'grasped_yaw_integration_validation.json').read_bytes())
    assert validation['passed'] and validation['tests_run']==232
    assert validation['previous_native_fixtures_passed']==21 and integration['passed']
    assert hashlib.sha256(CANDIDATE.read_bytes()).hexdigest()==CANDIDATE_SHA
    candidate=json.loads(CANDIDATE.read_bytes());proofs=[]
    for label in ('grasped_yaw_fixture','grasped_yaw_prior_fixture','grasped_yaw_open_fixture'):
        folder=OUT/label;plan=GraspedYawPlan(folder/'fresh_off_request.json')
        proof=validate_proof(plan,folder/'native_screen.json')
        proofs.append(dict(fixture=label,static_cases=len(proof['static_screens']),dynamic_cases=len(proof['dynamic_sensitivity']),
            maximum_tracking_rad=max(s['max_tracking_rad'] for s in proof['dynamic_sensitivity']),
            proof_sha256=hashlib.sha256((folder/'native_screen.json').read_bytes()).hexdigest()))
    record=dict(status='operator_small_yaw_observation_ready_fresh_live_proof_required',
        operator_report_original='걸려서 빼다가 좀 움직였을걸',
        operator_report_interpretation='Subjective retention while removing the cube, not measured grasp force.',
        removal_motion_counted_as_rotation=False,previous_grasp_command_complete=True,
        previous_grip_strength_measured=False,physical_cube_rotation_verified=False,live_sac=False,
        selected_model_yaw_degrees=candidate['selected']['target_yaw_degrees'],
        positive_direction_candidate_rejected_for_middle_ring_contact=True,
        maximum_changed_joint_delta_ticks=64,maximum_step_ticks=2,
        turn_duration_s=4.5,before_fixed_hold_s=3.,after_fixed_hold_s=2.,
        frozen_turn_motor_ids=[4,8,12,13,14,15,16],current_or_PID_changed=False,
        native_joint_ranges_changed=False,original_slow_demo_changed=False,
        new_trial_motion_deadline_s=75,prior_slow_demo_motion_deadline_s=60,
        regression_tests=validation['tests_run'],new_native_fixtures=proofs,
        old_native_fixtures_verified=21,old_native_fixtures_replayed=validation['previous_native_fixtures_replayed'],
        validation_sha256=hashlib.sha256(validation_path.read_bytes()).hexdigest(),
        original_candidate_sha256=CANDIDATE_SHA,
        candidate_file_is_not_standalone_motion_authorization=True,
        next_launcher='leap_grasped_yaw_trial.cmd',
        user_action='Empty hand at first SPACE; ring separation SPACE; PLACE cube and hands clear SPACE; wait until REMOVE, then remove and SPACE.',
        requires_additional_photo_or_readonly_trial=False,
        assistant_opened_real_devices=False,assistant_hardware_commands_sent=0)
    (OUT/'grasped_yaw_ready_review.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
    title='## CURRENT 2026-10-23: 파지 걸림 보고 / 같은 파지에서 작은 회전 시험 준비'
    note='''

- 사용자는 18:21 파지 후 큐브를 뺄 때 걸렸고, 빼다가 손가락이 조금 움직였을 수 있다고 답했다. 이는 손가락의 걸림을 느꼈다는 관찰로 기록한다. 파지력이나 들어 올리기를 정량 검증한 것은 아니며, 제거 중 움직임을 큐브 회전 성공으로 세지 않는다. 추가 닫힘 후보는 실행하지 않고 방금 완료한 파지 끝점에서 작은 회전을 시험한다.
- 새 명령은 `leap_grasped_yaw_trial.cmd`다. 방금 성공한 빈손 준비와 약지 분리 확인, PLACE와 느린 파지를 사용한 뒤 3초간 같은 목표를 유지한다. 이어서 4.5초 동안 모델상 반대 방향 5도에 해당하는 작은 관절 경로를 명령하고 2초간 유지 및 사진을 저장한 뒤 REMOVE를 안내한다. 회전 구간에서 ID4, 8, 12와 약지 ID13~16은 고정하며, 나머지 관절도 기준에서 최대 64틱과 한 명령 최대 2틱으로 제한한다.
- 큐브의 주황 앞면 모서리를 REMOVE 안내 전에 관찰한다. 윗면 색이 그대로여도 수평 회전은 가능하다. 영상 판단에는 `stage_4_fixed_grasp_observation`, `stage_6_yaw_fixed_observation`과 `grasped_yaw_observation`만 사용한다. REMOVE 표본이나 OFF 뒤 처짐으로 회전 성공을 판정하지 않는다. 실제 결과의 회전 성공 필드는 계속 false로 둔다.
- 최초 반대쪽 후보는 중지 마디와 약지 근위 구조의 겹침이 생겨 제외했다. 해당 후보와 실패 증명은 보존했다. 채택한 반대 방향 후보는 관측한 추종 오차를 쓴 기하와 원시 명령 기하에서 모두 새 손 접촉과 받침 접촉이 없었다. 기본 관절 범위, 마커 장착 좌표와 PID 및 전류를 변경하지 않았다. 좌표와 접촉 추정의 불확실성은 유지하며 작은 관절 경로의 실제 결과를 별도로 관찰한다.
- 장치 없는 회귀 검사 232개를 통과했다. 세 가지 실제 저장 시작마다 67개 정적 및 3개 응답 검사를 통과했다. 기존 느린 파지 시작도 현재 검사 코드로 다시 통과했고, 기존 21개 경로도 새 검사 등록에 맞춰 모두 검증했다. 실제 실행 때는 새로 읽은 OFF 자세와 ±4틱 주변의 증명이 반드시 다시 필요하다. 과거 실물 실행 폴더의 원본 증명은 수정하지 않았다.
- 추가 동작과 관찰을 포함하는 새 실행만 75초의 동작 제한을 사용한다. 기존 느린 파지 시연의 60초 제한과 전압, 전류, 추종 및 통신 정지 조건은 그대로다. 목표와 증거 파일은 OFF 상태에서 읽어 메모리에 보관하며 ON 상태에서는 파일을 다시 읽지 않는다. 카메라는 별도 프로세스로 녹화하고 동작 중 마커 가림은 정지 조건으로 쓰지 않는다.
- **지금 사용자 작업:** 큐브를 먼저 빼고 `leap_grasped_yaw_trial.cmd`를 실행한다. 빈손 첫 SPACE, 약지 분리 확인 SPACE, PLACE 때 초록 위와 주황 카메라 쪽으로 큐브를 놓고 손을 뺀 뒤 SPACE. 자동 회전과 관찰이 끝나 REMOVE가 나올 때까지 만지지 않는다. 큐브 제거 후 SPACE. 잘 빠지지 않으면 Q로 토크를 끈 뒤 빼며, 제거 전 관찰은 이미 저장돼 있다. 끝나면 했어만 알려주면 저장된 원본을 읽는다.
- 옆 질문 처리: 1번 참가 신청서 소속은 기존 자료의 학과명인 컴퓨터공학과라고 답했다. 이 질문은 로봇 작업 중단 요청이 아니므로 경로 검토를 계속했다. 전시 문구는 수정하지 않았다.
- 근거: `right_hand_rotation_20261023/grasped_yaw_ready_review.json`, `grasped_yaw_validation.json`, `grasped_yaw_integration_validation.json`, `same_grasp_small_yaw_candidate.json`, `leap_grasped_yaw_trial_README.md`.

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8')
        if old.startswith(title):continue
        if old.startswith('## CURRENT '):old=old.replace('## CURRENT ','## PREVIOUS ',1)
        path.write_text(title+note+old,encoding='utf-8')
    print(json.dumps(record,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
