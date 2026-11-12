import json
from pathlib import Path
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/lower_body_regrasp'
p=O/'free_continuation_review.json'
if p.exists():
 r=json.loads(p.read_bytes());print('NOMINAL',r['safety_route_passed'],r['body_hold_task_all_probes_passed'],r['worst'])
 for s in r['samples']:
  if s['phase'] in ('stage_13_body_close','stage_14_body_hold','stage_9_loaded_open','stage_10_remove_hold'):
   print(s['sample'],s['phase'],round(s['yaw_change_deg'],3),round(s['center_change_mm'],3))
for f in (-15,-10,-5):
 p=O/f'full_front_{f}/review.json'
 if p.exists():
  r=json.loads(p.read_bytes());print('FULL',f,len(r['cases']),r['all_passed'],[c['body_hold_observation']['hold_task_passed'] for c in r['cases']])
