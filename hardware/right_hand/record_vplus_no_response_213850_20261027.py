"""Record post-refastening read-only failure; leave motion instructions disabled."""
import datetime as dt,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
source=ROOT/'all_ping_readonly_20261027_213850_077952'/'result.json'
read=json.loads(source.read_text(encoding='utf-8'))
assert read['status']=='all_known_ids_pinged_read_only' and read['responding_ids']==[] and read['motor_writes']==0
folder=ROOT/'right_hand_rotation_20261027'/'vplus_disconnect_211439'
report=dict(timestamp=dt.datetime.now().astimezone().isoformat(),status='refastened_power_reported_on_no_motor_responses',
    user_reported_wire_refastened=True,user_reported_power_on=True,
    readonly_source=str(source.relative_to(ROOT)),host_bps=3000000,responding_ids=[],usb_com7_present=True,
    usb_latency_ms=1,motor_writes=0,motion_ready=False,
    inference='Power delivery or communication path is not verified; no conclusion from wire color.',
    next_user_action='Unplug PSU mains and provide a close-up showing V+/V- labels and wire clamps before further wiring or motion.')
(folder/'after_refastening_readonly.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
path=ROOT/'right_hand_rotation_20261027'/'horizontal_quarter_trial_v6_current'/'readiness.json'
ready=json.loads(path.read_text(encoding='utf-8'));ready.update(status=report['status'],next_user_command=None,
    next_user_command_ready=False,pending_user_input='PSU unplugged close-up showing V+/V- labels and secured wire.',
    next_user_task=report['next_user_action'],latest_connection_review=str((folder/'after_refastening_readonly.json').relative_to(ROOT)))
path.write_text(json.dumps(ready,indent=2)+'\n',encoding='utf-8')
header='''연결 재확인(2026-10-27 21:38):사용자가V+선을 다시 끼웠고 전원도 이미 켰다고 답함. COM7은 있으나 기존3Mbps에서ID1~16 PING응답0개. 모터/설정 쓰기0회. 전원 또는 통신 경로 복구가 아직 확인되지 않아 동작 명령 안내를 보류. 다음 사용자 작업은 전원장치 콘센트를 빼고V+/V-표시 및 선 고정부가 보이는 근접사진 제공. 근거 `right_hand_rotation_20261027/vplus_disconnect_211439/after_refastening_readonly.json`.

<!-- END VPLUS NO RESPONSE 2138 20261027 -->

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    p=ROOT/name;old=p.read_text(encoding='utf-8')
    if '<!-- END VPLUS NO RESPONSE 2138 20261027 -->' not in old:p.write_text(header+old,encoding='utf-8')
print(json.dumps(dict(status=report['status'],responding_motor_count=0,motor_writes=0),ensure_ascii=False))
