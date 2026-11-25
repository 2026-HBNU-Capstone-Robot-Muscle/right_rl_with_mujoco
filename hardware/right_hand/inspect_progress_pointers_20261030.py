import json
from pathlib import Path
R=Path(__file__).resolve().parent
for name in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json','right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
 r=json.loads((R/name).read_bytes());print(name,list(r.keys()))
 print(json.dumps({k:v for k,v in r.items() if any(s in k for s in ('command','ready','user_action','status')) and not isinstance(v,(dict,list))},ensure_ascii=False))
