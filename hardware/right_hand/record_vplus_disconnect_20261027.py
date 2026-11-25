"""Record the operator-reported loose V+ wire without changing devices."""
import datetime as dt,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
trial=ROOT/'horizontal_quarter_trial_20261027_211439_985362'
r=json.loads((trial/'result.json').read_text(encoding='utf-8'))
last=r['samples'][-1]
report=dict(timestamp=dt.datetime.now().astimezone().isoformat(),
    status='reported_vplus_wire_disconnected_repair_confirmation_pending',
    trial=trial.name,stopped_during='stage_3_empty_open',motion_seconds=last['t'],
    cube_placement_reached=False,rotation_reached=False,
    initial_read_failure=r['read_failures'][0],
    last_complete_voltage_range_v=[min(last['voltage_v']),max(last['voltage_v'])],
    last_complete_maximum_tracking_rad=max(last['tracking_rad']),
    camera_all705frames_usable=True,off_write_packets_sent=2,torque_off_confirmed_by_program=False,
    user_confirmed_motor_power_cut=True,
    user_observation='Black wire had come out of its V+ terminal; original terminal identity was reported by user, not inferred from wire color.',
    inference='Likely supply interruption; exact interrupted branch not electrically measured.',
    user_repair_instruction='Unplug PSU mains. Refasten only the wire originally in V+ to the same V+ terminal. Then switch5V back on and report; do not start a motion program.',
    next_step='After user confirms repair/power, perform PING/READ-only communication and torque-OFF checks first.',
    no_baud_or_usb_setting_change_needed_at_this_stage=True,
    assistant_motor_writes=0,next_motion_command_ready=False,
    physical_horizontal_90_verified=False)
folder=ROOT/'right_hand_rotation_20261027'/'vplus_disconnect_211439';folder.mkdir(exist_ok=True)
(folder/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
ready=ROOT/'right_hand_rotation_20261027'/'horizontal_quarter_trial_v6_current'/'readiness.json'
data=json.loads(ready.read_text(encoding='utf-8'));data.update(status=report['status'],
    latest_user_trial=trial.name,power_cable_review=str((folder/'review.json').relative_to(ROOT)),
    next_user_command_ready=False,next_user_command=None,
    pending_user_input='Original V+ wire refastened with PSU unplugged, then5V power restored.',
    next_user_task='Repair V+ connection with PSU unplugged; restore5V and notify. No motion program.',
    physical_horizontal_90_verified=False)
ready.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
header='''当前 상태更新(2026-10-27):21:14실행은 빈손 준비 OPEN 중 ID9이후 응답을 받지 못해 중단,프로그램의 토크OFF 확인도 실패. 사용자는5V전원을 껐다고 확인했고,이후 원래V+단자에서 검은 선이 빠져있었다고 보고함. 전원장치 콘센트를 빼고 그 선을 원래V+에 고정한 뒤5V전원을 다시 켜달라고 요청한 상태. 복구 확인 후 PING/READ-only 검사부터 진행하며 모터 프로그램 재실행은 아직 안내하지 않음. 큐브 PLACE/회전 단계 미도달. 근거 `right_hand_rotation_20261027/vplus_disconnect_211439/review.json`.

<!-- END VPLUS REPAIR PENDING 20261027 -->

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/name;text=path.read_text(encoding='utf-8')
    if '<!-- END VPLUS REPAIR PENDING 20261027 -->' not in text:path.write_text(header+text,encoding='utf-8')
print(json.dumps(dict(status=report['status'],motor_writes=0,motion_ready=False),ensure_ascii=False))
