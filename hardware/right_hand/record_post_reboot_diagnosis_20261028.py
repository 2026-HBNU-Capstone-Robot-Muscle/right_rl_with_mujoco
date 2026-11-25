"""Keep confirmed recovery readings separate from the earlier unsafe stop."""
import json,datetime as dt
from pathlib import Path
ROOT=Path(__file__).resolve().parent
folder=ROOT/'right_hand_rotation_20261028/loaded_comm_stop_000157'
ping_name='all_ping_readonly_20261028_003507_156482/result.json'
off_name='repeat_off_readonly_20261028_003857_315371/result.json'
stress_name='serial_packet_readonly_20261028_003756_038279/result.json'
ping=json.loads((ROOT/ping_name).read_bytes());off=json.loads((ROOT/off_name).read_bytes());stress=json.loads((ROOT/stress_name).read_bytes())
assert ping['responding_ids']==list(range(1,17)) and off['status']=='off_snapshots_saved'
assert stress['reads']==1000 and not stress['errors'] and stress['camera_frames']>0
record=dict(status='all_ids_recovered_stationary_camera_reads_clean_motion_fault_unresolved',
    timestamp=dt.datetime.now().astimezone().isoformat(),user_confirmed_cube_removed=True,
    user_confirmed_supply_reenabled=True,individual_ping=ping_name,stable_off=off_name,concurrent_camera_stress=stress_name,
    responding_motor_ids=list(range(1,17)),current_torque_off_verified=True,current_error_and_watchdog_zero=True,
    camera_concurrent_full_reads=1000,camera_frames=stress['camera_frames'],read_errors=0,
    stationary_median_read_ms=stress['median_ms'],stationary_test_does_not_verify_communication_during_motion=True,
    last_stop_first_missing_motor_id=9,motor_or_connector_failure_not_proven=True,
    host_bps=3000000,baud_or_EEPROM_changed=False,motor_writes=0,
    pending_user_action='Power supply OFF; check already-connected white3pin connector seating on middle fingertip ID9. Keep power OFF afterward.',
    motion_retry_ready=False)
path=folder/'post_reboot_diagnosis.json';path.write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
ready=ROOT/'right_hand_rotation_20261027/horizontal_quarter_trial_v8_receive_resync/readiness.json'
data=json.loads(ready.read_bytes());data.update(status='awaiting_power_off_ID9_connector_seating_check',
    next_user_command_ready=False,pending_user_input=record['pending_user_action'],
    latest_connection_review=path.relative_to(ROOT).as_posix(),current_torque_off_verified=True)
ready.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json',
             'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    target=ROOT/name
    if target.exists():
        item=json.loads(target.read_bytes());item.update(current_status=data['status'],next_user_command_ready=False,
            pending_user_input=data['pending_user_input']);target.write_text(json.dumps(item,indent=2)+'\n',encoding='utf-8')
marker='<!-- END POST REBOOT READONLY 20261028 -->'
header='''2026-10-28 개별 통신 확인: 사용자가 큐브를 빼고 전원을 다시 켰다. 16개 개별PING 모두응답, 현재토크OFF/오류0/워치독0/5.0~5.1V 확인. 웹캠 동시1000회 전체상태 읽기도 오류0, 카메라294프레임이었다. 이는 정지 상태 검사이며 동작 중 통신 장애가 해결된 것을 의미하지 않는다. 마지막 실패에서 처음 응답이 누락된 ID9(중지 끝)의 이미 꽂혀 있는 흰색3핀 잭 체결을 전원을 끈 상태에서 확인하도록 요청했다. 동작 재시도 보류, baud/EEPROM 변경 없음. 현재 근거 `right_hand_rotation_20261028/loaded_comm_stop_000157/post_reboot_diagnosis.json`.

'''+marker+'\n\n'
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    target=ROOT/name;old=target.read_text(encoding='utf-8')
    if marker not in old:target.write_text(header+old,encoding='utf-8')
print(json.dumps(dict(status=record['status'],responding=16,camera_full_reads=1000,errors=0,motor_writes=0,motion_retry_ready=False)))
