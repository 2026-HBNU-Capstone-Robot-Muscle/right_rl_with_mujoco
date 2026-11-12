"""Record the verified read-recovery gate and the latest actual stop."""
import hashlib
import json
from pathlib import Path
from grasped_yaw_plan import GraspedYawPlan
from slow_grasp_demo_plan import SlowGraspDemoPlan
from thumb_live_start_gate import validate_proof

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261023'


def main():
    validation=json.loads((OUT/'grasped_yaw_validation.json').read_bytes())
    assert validation['passed'] and validation['tests_run']==247
    assert validation['previous_native_fixtures_passed']==21
    stop=json.loads((ROOT/'grasped_yaw_trial_20261023_222816_628099/goal_stop_review.json').read_bytes())
    assert stop['unchanged_old_goal_reads']==4 and stop['all_torque_off_verified'] and stop['settings_restored']
    records=[]
    for label in ('grasped_yaw_fixture','grasped_yaw_prior_fixture','grasped_yaw_open_fixture',
                  'grasped_yaw_after_stop_fixture','grasped_yaw_goal_stop_fixture','slow_grasp_demo_fixture'):
        folder=OUT/label;kind=SlowGraspDemoPlan if label=='slow_grasp_demo_fixture' else GraspedYawPlan
        plan=kind(folder/'fresh_off_request.json');proof=validate_proof(plan,folder/'native_screen.json')
        records.append(dict(fixture=label,static_cases=len(proof['static_screens']),
            dynamic_cases=len(proof['dynamic_sensitivity']),
            maximum_tracking_rad=max(row['max_tracking_rad'] for row in proof['dynamic_sensitivity']),
            proof_sha256=hashlib.sha256((folder/'native_screen.json').read_bytes()).hexdigest()))
    record=dict(status='operator_trial_ready_with_post_read_recovery_confirmation',
        actual_latest_stop_source=stop['source'],physical_communication_root_cause_resolved=False,
        change='After a recovered read, require two additional clean exact-goal healthy whole-hand snapshots before the next target.',
        initial_plus_confirmation_budget_ms=200,goal_writes_during_confirmation=0,
        requires_opt_in_plan=True,other_plans_sampling_changed=False,
        goal_mismatch_stops_unchanged=True,persistent_saved_goal_failure_still_stops=True,
        goals_or_joint_ranges_changed=False,drive_or_health_limits_changed=False,
        helper_preloaded_before_motor_enable=True,read_recovery_module_sha256=hashlib.sha256((ROOT/'read_recovery_confirmation.py').read_bytes()).hexdigest(),
        regression_tests_passed=247,native_fixtures=records,legacy_native_fixtures_validated=21,
        legacy_native_fixtures_replayed=validation['previous_native_fixtures_replayed'],
        camera_exposure=-5.,physical_rotation_verified=False,live_sac=False,
        next_user_command='leap_bus_echo_off.cmd',additional_camera_capture_required=False,
        off_echo_diagnostic_tests_passed=10,
        off_echo_source_sha256=hashlib.sha256((ROOT/'bus_echo_off.py').read_bytes()).hexdigest(),
        off_echo_scope='Only Goal Position RAM +/-1 tick while every motor is already torque OFF; restore original goals.',
        off_echo_torque_or_drive_or_eeprom_writes=0,
        next_motion_trial_chosen_after_OFF_packet_diagnostic=True,
        raw_execution_results_modified=False,assistant_opened_real_devices=False)
    (OUT/'read_recovery_ready_review.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
    title='## CURRENT 2026-10-23 22:28: 파지 중 목표 미반영 중단 / 읽기 복구 후 확인 추가'
    note='''

- 최신 실물 실행 `grasped_yaw_trial_20261023_222816_628099`는 큐브를 놓은 뒤 파지 도중 중단됐고, 회전 구간에는 도달하지 않았다. 목표 레지스터 전체 벡터가 바로 이전 명령과 네 번 연속 일치했다. 현재 명령과 이전 명령 차이는 최대 2틱이며, 이는 물리적인 관절 추종 오차와 별개의 목표 전송 확인 실패다. 기존 150ms 및 최대 3회 추가 읽기 동안 목표를 다시 쓰거나 다음 목표를 보내지 않았다.
- 그 직전 동기 읽기에는 ID1 뒤 패킷 손상 -3002가 있었고 한 번의 새 읽기로 47ms 안에 복구됐다. 앞서 두 번의 시간 초과도 읽기 복구로 돌아온 기록이 있다. 복구 직후 다음 목표를 바로 전송한 순서는 확인되지만, 이 순서가 목표 미반영의 물리적 원인이라고 입증한 것은 아니다. USB나 배선 원인이 해결됐다고 주장하지 않는다.
- 실제 최소 전압 4.0V, 최대 절댓값 전류 85mA, 최대 추종 오차 약 0.06903rad. 모든 토크 OFF와 설정 및 카메라 노출 복원을 확인했다. 카메라 -5 적용 후 901장의 원본이 저장됐고 배치 사진에서 손 구조와 큐브를 구분할 수 있다. 추가 밝기 비교 촬영은 요청하지 않는다.
- `GraspedYawPlan`에만 읽기 복구 후 확인을 추가했다. 같은 목표를 유지하면서 새로운 전체 상태를 두 번 더 읽고, 기존 건강 검사 및 매 표본 관절 보호를 모두 통과해야 다음 목표로 간다. 최초 읽기와 복구 시간을 포함한 기존 200ms 한도 안에서만 수행한다. 확인 중 통신 오류가 다시 생기거나 목표, 전류, 전압, 온도, 토크, watchdog, 관절 상태가 다르면 중단한다. 목표 재전송, 임의의 목표 오차 허용, 경로 재시작은 추가하지 않았다. 사용 모듈은 모터 활성화 전에 미리 로드해 ON 상태에서 소스 파일을 읽지 않는다.
- 기존 코드의 지속적인 이전 목표를 허용하는 것이 아니다. 이번 실제 실패의 명령 벡터와 읽기 벡터를 재생한 검사에서도 계속 중단됨을 확인했다. 수정은 복구 직후 진행을 늦추고 새 상태 확인을 추가하는 대응이며, 실제 재현 결과가 필요하다.
- 장치 없는 검사 247개를 통과했다. 복구 확인 중 Q 및 읽기나 카메라 오류, 건강 상태 오류, 고정 관절 오류, 200ms 전체 예산 초과, 두 번째 통신 오류, 지속된 실제 목표 미반영을 확인했다. 전체 시험의 가짜 장치 검사에서는 읽기 표본만 두 개 늘고 목표 큐와 전송된 목표 목록은 그대로인 채 OFF와 설정 복원을 완료했다.
- 수정 코드에 맞는 67개 정적 및 3개 응답 검사를 세 기존 회전 시작과 두 실제 중단 뒤 OFF 끝점, 기존 느린 파지 시작에서 다시 확인했다. 다른 21개 경로도 현재 코드와 일치하는 증명을 검증했다. 과거 실물 실행 폴더의 원본 증명과 결과는 수정하지 않았다. 실제 실행에서는 새로 읽은 안정된 OFF 자세와 경로 증명이 항상 다시 필요하다.
- 다음 실물 동작을 고르기 전에 목표 패킷 전달 자체를 확인하는 `leap_bus_echo_off.cmd`를 준비했다. 모든 모터가 이미 토크 OFF임을 매 쓰기 전에 새로 확인한다. 저장된 Goal Position RAM 값에 최대 ±1틱의 시험값만 기록하고 원래 값을 복원한다. 토크, 전류, PID, 모드와 EEPROM은 쓰지 않는다. 15초는 카메라를 닫고, 15초는 자동으로 웹캠을 켜 같은 전달 검사를 하며, 정지나 오류 시에도 새 OFF 상태가 확인될 때만 원래 목표를 복원한다. 목표 재전송으로 오류를 덮지 않고 전달 실패를 기록한다.
- 이 진단의 장치 없는 별도 검사 10개를 통과했다. 실제 손 위치를 바꾸지 않고 목표만 복원하는 정상 두 조건, 처음부터 ON 상태 및 너무 느린 읽기의 쓰기 차단, 중간 ON 상태에서 복원 쓰기 차단, 목표 패킷 누락, Q와 카메라 오류, 지속 읽기 실패, 오래된 OFF 확인, 범위 밖 목표, SDK 패킷이 오직 주소116/4바이트와 ID1~16만 쓰는지, 카메라 조건에서 실제 새 프레임이 필요한지 확인했다. 추가 카메라 밝기나 마커 촬영은 필요 없다.
- **지금 사용자 작업:** 큐브를 빼고 다른 모터 및 카메라 프로그램을 닫는다. USB와 모터 전원을 연결한 뒤 `leap_bus_echo_off.cmd`를 실행하고 터미널에서 SPACE 한 번 누른다. 손가락을 다시 배치하거나 큐브를 놓지 않는다. 약 30초의 두 조건 검사가 끝나면 했어만 알려주면 된다. 새 결과 `bus_echo_off_날짜_시간/result.json`의 두 조건 오류 및 목표 복원 상태를 읽고 다음 회전 시험을 결정한다. 회전 프로그램의 재실행은 아직 요청하지 않는다.
- 실제 목표 방향 회전이나 실물 SAC 정책은 여전히 미검증이다. 근거는 `read_recovery_ready_review.json`, `grasped_yaw_validation.json`, `grasped_yaw_trial_20261023_222816_628099/goal_stop_review.json`과 각 네이티브 증명이다.

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        file=ROOT/name;old=file.read_text(encoding='utf-8')
        if old.startswith(title):continue
        if old.startswith('## CURRENT '):old=old.replace('## CURRENT ','## PREVIOUS ',1)
        file.write_text(title+note+old,encoding='utf-8')
    print(json.dumps(record,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
