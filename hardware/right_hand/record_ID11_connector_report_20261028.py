"""User-reported ID11 reseating; no assumption that motion is now reliable."""
import json,datetime as dt
from pathlib import Path
ROOT=Path(__file__).resolve().parent
folder=ROOT/'right_hand_rotation_20261028/loaded_comm_stop_000157'
record=dict(status='ID11_connector_reseated_user_report_awaiting_powered_reads',
    timestamp=dt.datetime.now().astimezone().isoformat(),
    user_report='11번이 빠져있던거같긴한데 다시 꼈거든 확인좀',
    reported_motor_connector=11,unplugged_state_certain=False,reseated_user_reported=True,
    camera='poweroff_connection_camera_20261028_004658_376883/result.json',
    full_connector_engagement_visually_verified=False,
    individual_ping='all_ping_readonly_20261028_004757_103178/result.json',responding_motor_ids=[],
    no_response_is_not_a_confirmed_hardware_failure=True,
    powered_retest_pending=True,communication_during_motion_verified=False,
    motion_retry_ready=False,motor_writes=0)
out=folder/'ID11_reseat_review.json';out.write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
ready=ROOT/'right_hand_rotation_20261027/horizontal_quarter_trial_v8_receive_resync/readiness.json'
data=json.loads(ready.read_bytes());data.update(status=record['status'],next_user_command_ready=False,
    current_torque_off_verified=False,pending_user_input='Supply ON after hands clear, for individual READs only',
    latest_connection_review=out.relative_to(ROOT).as_posix())
ready.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
marker='<!-- END ID11 CONNECTOR REPORT 20261028 -->'
header='''2026-10-28 커넥터 재체결 보고: 사용자가11번 커넥터가 빠져 있었던 것 같다며 다시 끼웠다고 말했다. 완전한 탈락 여부는 확정되지 않았다. 웹캠은 손이 일부 가려 체결 깊이를 검증하지 못했다. 당시16개PING 응답0으로, 전원을 다시 켠 뒤 개별 읽기를 요청한 상태다. 이 무응답을 고장으로 판정하지 않는다. 동작 프로그램 재실행 보류, 재체결 후 통신과 동작 신뢰성은 아직 검증 전이다. 현재 근거 `right_hand_rotation_20261028/loaded_comm_stop_000157/ID11_reseat_review.json`.

'''+marker+'\n\n'
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    p=ROOT/name;old=p.read_text(encoding='utf-8')
    if marker not in old:p.write_text(header+old,encoding='utf-8')
for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json',
             'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    p=ROOT/name
    if p.exists():
        value=json.loads(p.read_bytes());value.update(current_status=data['status'],next_user_command_ready=False,
            pending_user_input=data['pending_user_input']);p.write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(status=record['status'],motor_writes=0,motion_ready=False)))
