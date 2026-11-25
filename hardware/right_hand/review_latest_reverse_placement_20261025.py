"""Summarize saved failures only; does not access devices."""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
fixture=json.loads((ROOT/'right_hand_rotation_20261024/reverse_empty_entry/placement_fixture_v6/request.json').read_bytes())
reference=np.array(fixture['off_state']['position'])
for folder in sorted(ROOT.glob('reverse_start_placement_20261025_*')):
    r=json.loads((folder/'result.json').read_bytes())
    print(folder.name,json.dumps({k:r.get(k) for k in ('status','phase','error','all_torque_off_verified','settings_restored')},ensure_ascii=False),'writes',len(r.get('write_events',[])))
    frames=sorted((folder/'camera').glob('frame_*.jpg'))
    if frames:print('last_camera_frame',str(frames[-1]))
    if (folder/'fresh_off_request.json').exists():
        q=json.loads((folder/'fresh_off_request.json').read_bytes())['off_state']['position']
        print('fresh_position',q,'difference_from_development_input',(np.array(q)-reference).tolist())
    if (folder/'current_start_screen.json').exists():
        s=json.loads((folder/'current_start_screen.json').read_bytes())
        for d in s['dynamic_sensitivity']:
            if d['book_front_mm']==-5 and d['gain_scale']==.5:print('weak_nearest_book_case',json.dumps(d,ensure_ascii=False))
