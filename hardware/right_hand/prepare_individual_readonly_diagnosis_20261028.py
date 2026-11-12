"""Record current camera findings and the next READ-only connection check."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
review=ROOT/'right_hand_rotation_20261028/loaded_comm_stop_000157/review.json'
data=json.loads(review.read_bytes());data.update(
    current_camera='poweroff_connection_camera_20261028_002618_195918/result.json',
    current_camera_cube_visible=True,connector_clamping_not_visually_verified=True,
    next_user_task='Remove cube with supply OFF, then turn supply ON for PING/READ only. No motion program.',
    pending_user_input='cube removed and supply ON for individual motor reads')
review.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
ready=ROOT/'right_hand_rotation_20261027/horizontal_quarter_trial_v8_receive_resync/readiness.json'
item=json.loads(ready.read_bytes());item.update(status='waiting_for_individual_readonly_connection_check',
    next_user_command_ready=False,pending_user_input=data['pending_user_input'])
ready.write_text(json.dumps(item,indent=2)+'\n',encoding='utf-8')
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/name;old=path.read_text(encoding='utf-8')
    path.write_text(old.replace('2026-10-28 最新状态：','2026-10-28 현재 상태: ',1),encoding='utf-8')
for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json',
             'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/name
    if path.exists():
        d=json.loads(path.read_bytes());d.update(current_status=item['status'],next_user_command_ready=False,
            pending_user_input=item['pending_user_input']);path.write_text(json.dumps(d,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(status=item['status'],motion_ready=False,motor_commands=0)))
