"""Record verified loaded-start incident and the next concrete user action."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent


def main():
    folder=ROOT/'right_hand_rotation_20261023'
    incident_path=folder/'loaded_demo_start_review.json';incident=json.loads(incident_path.read_bytes())
    validation_path=folder/'slow_grasp_demo_validation.json';validation=json.loads(validation_path.read_bytes())
    native_path=folder/'demo_empty_start_native_review.json';native=json.loads(native_path.read_bytes())
    latest=incident['runs'][-1]
    assert latest['all_torque_off_verified'] and latest['settings_restored']
    assert not latest['loaded_grasp_started']
    assert validation['passed'] and validation['tests_run']==218
    assert all(f['accepted'] and f['static_cases']==67 and f['dynamic_cases']==3 for f in native['fixtures'])
    assert validation['existing_native_fixtures_passed']==21
    record=dict(status='loaded_start_reviewed_empty_start_gate_tested_ready_for_operator',
        latest_physical_run=latest['folder'],latest_physical_run_stopped_at='place_cube',
        empty_hand_precondition_satisfied=False,physical_grasp_demo_completed=False,
        physical_cube_rotation_verified=False,
        latest_run_torque_off_verified=True,latest_run_settings_restored=True,
        empty_start_blocked_on_actual_saved_cube_frame=True,minimum_distinct_cube_free_marker_frames=10,
        marker_absence_certifies_empty_hand=False,requires_operator_to_remove_cube=True,
        detection_only_before_configuration_and_enable=True,marker_loss_during_motion_is_stop=False,
        no_id40_requirement_after_placement=True,motor_targets_or_drive_limits_changed=False,
        regression_tests=validation['tests_run'],native_fixtures=native['fixtures'],
        existing_unchanged_native_fixtures_verified=21,
        existing_fixtures_replayed_this_review=validation['existing_fixtures_replayed'],
        incident_sha256=hashlib.sha256(incident_path.read_bytes()).hexdigest(),
        regression_report_sha256=hashlib.sha256(validation_path.read_bytes()).hexdigest(),
        next_launcher='leap_slow_grasp_demo.cmd',
        next_user_action='Remove cube from hand and camera view before first SPACE. Place only at PLACE CUBE; remove at REMOVE CUBE.',
        assistant_opened_real_camera_or_motor_port=False,assistant_hardware_commands_sent=0)
    path=folder/'demo_empty_start_fix_review.json'
    path.write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
    title='## CURRENT 2026-10-23 17:52 실행 검토: 큐브 조기 배치 차단 수정'
    note='''

- 사용자가 처음부터 큐브를 올렸다고 확인했다. `slow_grasp_demo_20261023_175232_779550`의 시작 사진과 약지 분리 및 열린 자세 사진에서도 큐브 마커를 직접 해독했다. 이 실행은 빈손 준비라는 전제를 충족하지 않아 빈손 검증으로 사용하지 않는다.
- 실제 진행은 약지 분리, 엄지 이동, 손 열기와 `PLACE CUBE` 대기까지다. 사용자 중단으로 끝났고 느린 파지 및 3초 관찰은 실행하지 않았다. 최종 16개 토크 OFF와 설정 복원을 확인했다. 저장 표본의 최저 전압은 4.0V, 최대 절대 전류는 87mA, 최대 추종 오차는 약 0.0614rad였으며 하드웨어 오류는 없었다. 이 수치만으로 실물 접촉 부재나 손상 부재를 보증하지 않는다.
- 새 시작 검사는 첫 SPACE 전에 큐브 ID0~5가 해독되면 모터 연결 전에 차단한다. 큐브를 손과 카메라 화면 밖으로 빼고 새 프레임 10장 동안 큐브 마커가 해독되지 않으면 다시 시작할 수 있다. OFF 읽기, 경로 검사와 카운트다운에서도 큐브를 감지하면 동작 전에 중단한다. 마커 부재를 빈손의 증명으로 취급하지 않으며 사용자는 실제로 큐브를 빼야 한다.
- 이 검사에는 손바닥 ID40 인식이 필요 없다. 모터 설정 시작 이후에는 추가 해독을 수행하지 않는다. PLACE 이후 ID40이나 큐브 마커가 가려져도 이 검사로 중단하지 않는다. 기존 전류, PID, 경로 목표, 속도 및 상태 중단 기준은 변경하지 않았다.
- 실제 저장 시작 사진으로 모터 연결 전 차단을 확인하는 시험을 포함해 장치 없는 검사 218개가 통과했다. 처음 시작과 이번 종료 후 OFF 자세 두 개에 대해 각각 67개 정적 및 3개 응답 MuJoCo 검사를 새로 수행해 통과했다. 이번 OFF 자세의 고정 접촉과 받침 접촉은 모델에서 없었다. 기존 21개 증명도 현재 코드 해시와 전체 검증 조건이 그대로 맞는지 확인했다. 해당 21개는 직전 검토에서 실행한 증명을 재사용한 것이며 이번에 새로 실행했다고 표기하지 않는다.
- **지금 필요한 사용자 작업:** 손가락은 그대로 두고 큐브만 손과 카메라 화면 밖으로 뺀다. `leap_slow_grasp_demo.cmd` 실행 후 빈손으로 첫 SPACE. 약지 분리 확인에서 SPACE. `PLACE CUBE NOW`가 나온 뒤 초록 위와 주황 카메라 쪽으로 큐브를 놓고 손을 뺀 뒤 SPACE. `REMOVE CUBE` 때 큐브를 빼고 SPACE. 끝나면 했어만 알려주면 저장 결과를 읽는다.
- 실제 목표 방향 회전과 SAC 실물 제어는 아직 확인되지 않았다. 전시 문구의 완성 시점 가정과 실제 실험 상태를 구분한다. 추가 읽기 전용 촬영이나 로그 전체 붙여넣기를 요구하지 않는다.
- 근거: `right_hand_rotation_20261023/demo_empty_start_fix_review.json`, `loaded_demo_start_review.json`, `demo_empty_start_native_review.json`, `slow_grasp_demo_validation.json`, `leap_slow_grasp_demo_README.md`.

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        destination=ROOT/name;old=destination.read_text(encoding='utf-8')
        if old.startswith(title):continue
        if old.startswith('## CURRENT '):old=old.replace('## CURRENT ','## PREVIOUS ',1)
        destination.write_text(title+note+old,encoding='utf-8')
    print(json.dumps(record,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
