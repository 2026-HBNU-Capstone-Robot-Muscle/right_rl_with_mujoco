"""Inspect saved endpoints and path assets; no device access."""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
r=json.loads((ROOT/'fresh_empty_grasp_20261027_032949_661470/result.json').read_bytes())
print('RESULT',json.dumps({k:r.get(k) for k in ('plan_version','accepted_enable_goal','planned_final_command','final_command','final_off_reference_valid','final_off_endpoint_drift_ticks','on_to_off_position_change_ticks')},indent=2))
print('ON',json.dumps({k:r.get('final_on_state',{}).get(k) for k in ('position','goal','voltage','current')}))
for i,s in enumerate(r.get('final_off_snapshots',[])):print('OFF',i,json.dumps({k:s[k] for k in ('position','goal','voltage','torque','error','watchdog')}))
p=ROOT/'right_hand_rotation_20261026/fresh_empty_start_20261026_235258/recovery_plan_v2_ring_first/path.npz';z=np.load(p);print('NPZ KEYS',z.files)
for key in z.files:
    a=z[key]
    if key=='phases':print(key,[(str(v),int(np.sum(a==v))) for v in np.unique(a)])
    elif a.ndim<=1:print(key,a.tolist())
    else:print(key,a.shape,'first',a[0].tolist(),'last',a[-1].tolist())
for name in ('right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json','right_hand_rotation_20261026/legacy_stage4_ring_uncurl_other_v1/approach_review.json'):
    p=ROOT/name
    if p.exists():
        obj=json.loads(p.read_bytes());print(name,json.dumps(obj,indent=2)[:10000])
