"""Print only compact, saved offline near-quarter evidence."""
import json
from pathlib import Path
O=Path(__file__).resolve().parent/'right_hand_rotation_20261105/canonical_body_grip'
for name in ('postturn_regrasp/review.json','near_quarter_full_replay/review.json'):
    p=O/name
    if not p.exists():continue
    r=json.loads(p.read_bytes())
    if 'cases' in r:
        for c in r['cases']:
            print(json.dumps(dict(finger=c['finger'],mode=c['mode'],static=c['static_proposal_accepted'],contacts=[{k:v for k,v in t.items() if k!='point'} for t in c['contacts']],self_mm=c['self_mm'],support_mm=c['support_mm'],cube_mm=c['cube_mm'],dynamic_pass=[v.get('result',{}).get('whole_grip_task_passed',False) for v in c['dynamic']])))
    else:
        d=r['dynamic'];print(json.dumps(dict(full_initial_replay=True,safety=d['safety_passed'],yaw=d['final_yaw_from_placement_deg'],tilt=d['final_up_tilt_deg'],hold=d['full_fixed_hold'],tail=d['settled_tail'],task_reasons=r['quarter_and_common_grip_reasons'])))
