import json
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parent/'right_hand_rotation_20261023'
folder=root/'grasped_yaw_LED_complete_fixture'
request=json.loads((folder/'request.json').read_bytes());q=np.array(request['off_state']['position'])
print('latest_start',q.tolist())
for name in ('grasped_yaw_fixture','grasped_yaw_prior_fixture','grasped_yaw_open_fixture','grasped_yaw_after_stop_fixture','grasped_yaw_goal_stop_fixture'):
    old=np.array(json.loads((root/name/'fresh_off_request.json').read_bytes())['off_state']['position'])
    print(name,{i+1:int(q[i]-old[i]) for i in range(16) if q[i]!=old[i]})
path=sorted(folder.glob('windows_wsl_screen_*.json'))[-1];r=json.loads(path.read_bytes())
print('accepted',r['accepted'],'last_goal',r['last_goal_ticks'])
for label in ('static_screens','dynamic_sensitivity'):
    rows=r[label]
    print(label,len(rows))
    for i,row in enumerate(rows):
        if i==0 or label=='dynamic_sensitivity' or row.get('self_escape_max_unapproved_overlap_mm',0)>1e-6 or not row.get('self_escape_complete',True):
            print(i,row)
print('LOG',path.with_suffix('.log').read_text(encoding='utf-8',errors='replace')[-3000:])
