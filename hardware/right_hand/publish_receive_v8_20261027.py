"""Latest communication diagnosis and bounded-recovery operator handoff."""
import json,datetime as dt
from horizontal_quarter_plan_20261027 import ROOT,OUT,REQUEST,PROOF,HorizontalQuarterPlan,hashes
from horizontal_quarter_gate_20261027 import validate_review
proof=validate_review(HorizontalQuarterPlan(REQUEST),PROOF)
verification=json.loads((OUT/'verification.json').read_bytes())
assert verification['tests_passed']==113 and verification['code_hashes']==hashes()
source=ROOT/'repeat_off_readonly_20261027_234535_573422/result.json';off=json.loads(source.read_bytes())
stress_path=ROOT/'serial_packet_readonly_20261027_230709_595231/result.json';stress=json.loads(stress_path.read_bytes())
assert stress['reads']==1000 and not stress['errors'] and stress['servo_writes']==0
diagnosis=dict(latest_trial='horizontal_quarter_trial_20261027_225914_101070/result.json',
    primary_failure='Two consecutive COMM_RX_CORRUPT full health reads during EMPTY OPEN; no loaded motion',
    earlier_recovery='One earlier corrupt read recovered plus two fresh healthy snapshots within93ms',
    cleanup_failure='Torque OFF verified, then watchdog reset readback failed and hid the original read fault',
    minimum_trial_voltage_v=4.9,maximum_trial_current_ma=124,maximum_tracking_rad=.10737865515199488,
    all_torque_off_verified=True,settings_restored=False,underlying_motion_time_transport_fault_not_proven_resolved=True,
    latest_off_readonly=source.relative_to(ROOT).as_posix(),off_drift_ticks=off['maximum_drift_ticks'],
    stationary_readonly_stress=stress_path.relative_to(ROOT).as_posix(),stationary_full_reads=1000,stationary_errors=0,
    host_baud=3000000,servo_baud_or_EEPROM_changed=False,assistant_motor_writes=0)
(OUT/'latest_stop_review.json').write_text(json.dumps(diagnosis,indent=2)+'\n',encoding='utf-8')
ready=dict(status='v8_bounded_receive_recovery_ready',timestamp=dt.datetime.now().astimezone().isoformat(),
    latest_stop_review=(OUT/'latest_stop_review.json').relative_to(ROOT).as_posix(),
    latest_off_readonly=source.relative_to(ROOT).as_posix(),current_source=REQUEST.relative_to(ROOT).as_posix(),
    operator_proof=PROOF.relative_to(ROOT).as_posix(),tests_passed=113,current_model_cases=210,
    read_resynchronization_policy=proof['read_resynchronization_policy'],
    stable_placement_policy=proof['stable_placement_review']['policy'],
    loaded_goals_unchanged=True,hardware_health_thresholds_unchanged=True,
    hardware_motion_with_new_receive_policy_verified=False,underlying_connection_fault_proven_fixed=False,
    physical_horizontal_90_verified=False,assistant_motor_writes=0,
    next_user_command='leap_horizontal_quarter_trial.cmd',next_user_command_ready=True,pending_user_input=None,
    next_user_task='Leave fingers as they are. Close Wizard. EMPTY first SPACE. PLACE green up/orange toward wrist ID41, hands away/SPACE. After automatic OPEN remove cube/third SPACE. Send done only.')
(OUT/'readiness.json').write_text(json.dumps(ready,indent=2)+'\n',encoding='utf-8')
(OUT/'operator_guide.md').write_text('''# 通信 오류 복구 보완 후 실행

손가락은 현재대로 두고 위자드와 다른 모터 프로그램을 닫은 뒤 `leap_horizontal_quarter_trial.cmd`를 한 번 실행한다.

1. 큐브를 손과 카메라 밖에 두고 터미널에서 첫 SPACE를 누른다.
2. PLACE CUBE 때 초록 위, 주황 손목 ID41 방향으로 놓는다. 손을 뺀 뒤 두 번째 SPACE를 누른다.
3. 회전 목표와 자동 OPEN이 끝나면 큐브를 빼고 세 번째 SPACE를 누른다.

22:59 실행은 빈손 OPEN 중 전체 상태 읽기가 두 번 연속 손상돼 중단됐다. 직전 전압은4.9~5.0V였고 최대전류124mA, 최대추종오차0.107rad였다. 같은 실행의 앞선 수신 오류는 정상 읽기 두 번을 확인하고93ms 안에 복구했다. 마지막에는 OFF가 확인된 뒤 설정복구 읽기도 실패해 최종 오류 문구가 원래 통신 오류를 가렸다. 설정복구 완료는 확인되지 않았으며 현재 읽기에서는 모두 OFF, 오류0, 워치독0이다.

v8은 전체 상태 읽기(64번부터83바이트)에만 짧은 수신 재동기화를 적용한다. 실패한 응답의 나머지 바이트를 읽어서 정리하고3ms 동안 새 바이트가 없을 때 다음 읽기를 보낸다. 정리는 최대20ms, 전체 읽기200ms, 최대3요청이며 기존10초당3회 재시도 제한을 유지한다. 복구 후에도 정확한 같은 목표와 정상 상태가 새 패킷 두 개에서 확인돼야 다음 목표를 보낸다. 목표 재전송과 건너뛰기는 없다. 지속 수신 오류, 건강 이상, 취소는 정지한다. 원래 동작 오류와 OFF 후 복구 오류를 따로 기록한다.

정지 상태1000회 읽기는 오류0이었다. 현재 자세 모델210조건과 제어 테스트113개 통과. 모터 속도, EEPROM, 힘, PID, 건강 차단 기준, 실제 목표 경로를 바꾸지 않았다. 새 처리의 실물 동작 검증과 수평90도 성공은 다음 실행의 저장 영상으로 확인해야 한다. 배선이나 움직일 때의 통신 문제가 해결됐다고 확인된 것은 아니다.
''',encoding='utf-8')
marker='<!-- END BOUNDED RECEIVE V8 20261027 -->'
header='''통신 복구 보완 완료(2026-10-27): 22:59 실행의 실제 중단은 빈손 OPEN 중 전체 모터 상태 패킷의 연속 수신 오류다. 전압최저4.9V, 최대전류124mA. OFF는 확인됐으나 후속 설정복구 읽기도 실패해 원래 오류를 가렸다. 현재16개 OFF/오류0/워치독0, 정지 상태1000회 읽기 오류0. v8은 같은 목표를 유지하고 전체 읽기200ms 안에서 수신 재동기화와 최대3요청을 허용한다. 복구 후 두 개의 새 정상 상태를 요구하며 전압, 전류, 추종오차와 기존10초 재시도 제한은 유지한다. 모델210조건과 테스트113개 통과. 실물 동작 중 통신 복구와 수평90도는 아직 확인 전이다. 다음 명령은 같은 `leap_horizontal_quarter_trial.cmd`. 빈손 첫SPACE, PLACE에서 초록위/주황손목쪽 후 손을 빼고SPACE, 자동OPEN 후 큐브제거/마지막SPACE. ‘했어’만 보내면 저장 파일을 검토한다. 현재 근거 `right_hand_rotation_20261027/horizontal_quarter_trial_v8_receive_resync/readiness.json`.

'''+marker+'\n\n'
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/name;old=path.read_text(encoding='utf-8')
    if marker not in old:path.write_text(header+old,encoding='utf-8')
for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json',
             'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/name
    if path.exists():
        data=json.loads(path.read_bytes());data.update(current_status=ready['status'],
            current_hand_review=(OUT/'readiness.json').relative_to(ROOT).as_posix(),
            next_user_command=ready['next_user_command'],next_user_command_ready=True,pending_user_input=None)
        path.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
previous=ROOT/'right_hand_rotation_20261027/horizontal_quarter_trial_v7_stable_placement/readiness.json'
if previous.exists():
    old=json.loads(previous.read_bytes());old.update(status='superseded_by_v8_receive_recovery',next_user_command_ready=False,
        superseded_by=(OUT/'readiness.json').relative_to(ROOT).as_posix());previous.write_text(json.dumps(old,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(status=ready['status'],tests=113,motor_writes=0,next_user_command=ready['next_user_command'])))
