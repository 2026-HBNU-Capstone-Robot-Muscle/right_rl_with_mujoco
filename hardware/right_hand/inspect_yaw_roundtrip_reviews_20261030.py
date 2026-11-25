import json
from pathlib import Path
from yaw_roundtrip_plan_20261030 import OUT,MODEL_SOURCES
for p in [OUT/'loaded_hand_dynamics/review.json',OUT/'empty_recovery_screen/review.json',*MODEL_SOURCES]:
    try:r=json.loads(p.read_bytes())
    except (OSError,json.JSONDecodeError) as e:print(p.parent.name,type(e).__name__);continue
    cases=r.get('cases',[]);failed=[dict(front=c.get('book_front_mm'),gain=c.get('gain_scale'),mu=c.get('friction'),stop=c.get('stop'),initial=c.get('initial_placement_probe')) for c in cases if not c.get('passed')]
    inverse=[c['inverse_from_forward_hold_deg'] for c in cases if c.get('inverse_from_forward_hold_deg') is not None]
    print(json.dumps(dict(folder=p.parent.name,all_passed=r.get('all_passed'),cases=len(cases),failed=failed,
        inverse_range=[min(inverse),max(inverse)] if inverse else None,
        nominal=[dict(forward=c['forward_from_placement_deg'],inverse=c['inverse_from_forward_hold_deg']) for c in cases if (c.get('gain_scale'),c.get('friction'))==(1.,.3)])))
