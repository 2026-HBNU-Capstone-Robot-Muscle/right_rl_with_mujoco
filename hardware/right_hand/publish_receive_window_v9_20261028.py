"""Handoff after reported ID11 reseating and a bounded host receive update."""
import json,datetime as dt
from horizontal_quarter_plan_20261027 import ROOT,OUT,REQUEST,PROOF,HorizontalQuarterPlan,hashes
from horizontal_quarter_gate_20261027 import validate_review
proof=validate_review(HorizontalQuarterPlan(REQUEST),PROOF)
verification=json.loads((OUT/'verification.json').read_bytes())
assert verification['tests_passed']==122 and verification['code_hashes']==hashes()
ping_name='all_ping_readonly_20261028_005912_638817/result.json'
old_name='serial_packet_readonly_20261028_005916_400289/result.json'
new_name='sync_window_readonly_20261028_010733_403317/result.json'
ping=json.loads((ROOT/ping_name).read_bytes());old=json.loads((ROOT/old_name).read_bytes());new=json.loads((ROOT/new_name).read_bytes())
assert ping['responding_ids']==list(range(1,17)) and new['reads']==1000 and not new['errors'] and new['camera_frames']>0
ready=dict(status='v9_post_ID11_reseat_receive_window_ready',timestamp=dt.datetime.now().astimezone().isoformat(),
    user_reported_possible_loose_connector=11,reseating_reported=True,connector_engagement_visually_verified=False,
    current_individual_ping=ping_name,responding_motor_ids=list(range(1,17)),current_torque_off_verified=True,
    current_error_and_watchdog_zero=True,old_window_camera_stress=old_name,old_window_complete_reads=999,old_window_read_errors=1,
    last_partial_reply=dict(packet_id=14,expected_bytes=94,received_bytes=54,elapsed_ms=39.31840002769604,
        delayed_or_lost_remaining_bytes_not_distinguished=True),
    new_window_camera_stress=new_name,new_window_complete_reads=1000,new_window_read_errors=0,
    new_window_camera_frames=new['camera_frames'],new_window_median_ms=new['median_ms'],
    new_window_max_ms=new['max_ms'],sync_receive_window_policy=proof['sync_receive_window_policy'],
    finite_read_budget_not_disabled=True,loaded_goals_unchanged=True,hardware_health_thresholds_unchanged=True,
    current_source=REQUEST.relative_to(ROOT).as_posix(),operator_proof=PROOF.relative_to(ROOT).as_posix(),
    current_empty_static_cases=201,current_empty_dynamic_cases=9,tests_passed=122,
    physical_motion_after_reseating_verified=False,underlying_fault_proven_resolved=False,
    physical_horizontal_90_verified=False,baud_or_EEPROM_changed=False,assistant_motor_writes=0,
    next_user_command='leap_horizontal_quarter_trial.cmd',next_user_command_ready=True,pending_user_input=None,
    next_user_task='Leave fingers as now. Close Wizard. EMPTY first SPACE. PLACE between thumb and middle at roots: green up/orange toward wrist ID41; hands clear then SPACE. After turn and automatic OPEN, remove cube and third SPACE. Send done only.')
(OUT/'readiness.json').write_text(json.dumps(ready,indent=2)+'\n',encoding='utf-8')
(OUT/'operator_guide.md').write_text('''# 재체결 후 수평 회전 시험

손가락은 현재대로 두고 위자드와 다른 모터 프로그램을 닫은 뒤 `leap_horizontal_quarter_trial.cmd`를 한 번 실행한다.

1. 큐브를 손과 카메라 밖에 두고 첫 SPACE를 누른다.
2. PLACE CUBE 때 엄지와 중지 사이, 손가락 뿌리 쪽 빈 공간에 큐브를 자연스럽게 올린다. 초록 위, 주황은 손목 ID41 방향이다. 사람 손을 뺀 뒤 두 번째 SPACE를 누른다.
3. 회전 목표와 자동 OPEN이 끝나면 큐브를 빼고 세 번째 SPACE를 누른다. 결과는 ‘했어’만 보내면 된다.

사용자는11번 잭이 빠져 있었던 것 같다며 재체결했다고 보고했다. 재체결 뒤16개 개별 응답, 토크OFF, 오류0, 워치독0을 확인했다. 기존 수신 창에서 웹캠 동시1000회 중1회는14번 응답94바이트 중54바이트만 수신한 채 약39ms에 종료됐다. 나머지가 지연된 것인지 유실된 것인지는 이 기록만으로 구분되지 않는다.

v9은 전체 상태 읽기(64번부터83바이트)와 토크OFF 읽기(64번1바이트)에만 호스트 수신 창80ms를 적용한다. 정상 응답은 도착 즉시 처리하므로 매번80ms를 기다리지 않는다. 전체 건강 확인200ms, 복구 후 두 개의 새 정상 상태, 재시도와 전압 및 전류 및 추종오차 기준은 그대로다. 누락되거나 손상된 데이터는 승인하지 않는다. 설정이나 목표 쓰기 재시도는 없다.

새 방식으로 웹캠 동시1000회 읽기 오류0, 현재 자세 복귀 모델210조건과 제어 테스트122개 통과. 모터 baud와EEPROM, 힘과PID, 원래 실제 목표 경로는 변경하지 않았다. 이러한 정지 상태 검사가 움직일 때의 연결 안정성이나 실물 수평90도를 입증하지는 않는다. 다음 실물 실행으로 확인한다.
''',encoding='utf-8')
marker='<!-- END POST RESEAT WINDOW V9 20261028 -->'
header='''2026-10-28 재체결 후 시험 준비: 사용자가11번 잭이 빠져 있었던 것 같다며 재체결했다.16개 개별응답/토크OFF/오류0/워치독0 확인. 기존 수신 창의 웹캠 동시1000회에서1회 부분응답(94중54바이트,약39ms)이 남아, 호스트 수신 창을 전체 상태 및OFF 읽기에만80ms로 조정했다. 전체 건강 확인200ms와 기존 차단 기준 유지. 새 방식 웹캠 동시1000회 오류0, 현재 자세 모델210조건, 테스트122개 통과. baud/EEPROM/모터힘/PID/실제 목표 경로 변경 없음. 움직일 때의 통신과 실물 수평90도는 아직 검증 전이다. 다음은 같은 `leap_horizontal_quarter_trial.cmd`: 빈손 첫SPACE, PLACE에서 엄지와 중지 사이 뿌리 쪽에 초록위/주황손목방향으로 놓고 손을 뺀 뒤SPACE, 자동OPEN 후 큐브제거/마지막SPACE. 현재 근거 `right_hand_rotation_20261028/horizontal_quarter_trial_v9_receive_window/readiness.json`.

'''+marker+'\n\n'
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/name;previous=path.read_text(encoding='utf-8')
    if marker not in previous:path.write_text(header+previous,encoding='utf-8')
for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json',
             'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/name
    if path.exists():
        value=json.loads(path.read_bytes());value.update(current_status=ready['status'],
            current_hand_review=(OUT/'readiness.json').relative_to(ROOT).as_posix(),
            next_user_command=ready['next_user_command'],next_user_command_ready=True,pending_user_input=None)
        path.write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')
old_ready=ROOT/'right_hand_rotation_20261027/horizontal_quarter_trial_v8_receive_resync/readiness.json'
if old_ready.exists():
    value=json.loads(old_ready.read_bytes());value.update(status='superseded_by_v9_receive_window',next_user_command_ready=False,
        superseded_by=(OUT/'readiness.json').relative_to(ROOT).as_posix())
    old_ready.write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(status=ready['status'],tests=122,motor_writes=0,next_user_command=ready['next_user_command'])))
