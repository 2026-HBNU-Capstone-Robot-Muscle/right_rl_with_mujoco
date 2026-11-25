import json
from pathlib import Path
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/natural_cradle_grasp';rows=[]
for front in (-15,-10,-5):
 p=O/f'free_front_{front}/review.json'
 if not p.exists():continue
 r=json.loads(p.read_bytes())
 for c in r['cases']:rows.append(dict(front=front,gain=c['gain_scale'],friction=c['friction'],safety_passed=c['passed'],hold=c['grasp_hold_observation']))
print(json.dumps(dict(cases=len(rows),safety_passed=sum(r['safety_passed'] for r in rows),task_passed=sum(r['hold']['passed'] for r in rows),rows=rows),indent=2))
