import json,hashlib
from pathlib import Path
R=Path(__file__).resolve().parent
for name in ('lower_body_regrasp','lower_body_gap_release'):
 o=R/'right_hand_rotation_20261030'/name;p=o/'loaded_hand_dynamics/review.json'
 if p.exists():
  r=json.loads(p.read_bytes());print(name,dict(bound=r['source_sha256']==hashlib.sha256((o/'candidate.npz').read_bytes()).hexdigest(),cases=len(r['cases']),all_passed=r['all_passed']))
for f in (-15,-10,-5):
 p=R/f'right_hand_rotation_20261030/lower_body_regrasp/full_front_{f}/review.json'
 if p.exists():
  r=json.loads(p.read_bytes());print('FULL',f,dict(cases=len(r['cases']),all_safety=r['all_passed'],all_body_hold=all(c['body_hold_observation']['hold_task_passed'] for c in r['cases']),max_hold_angle=max(c['body_hold_observation']['max_yaw_drift_deg'] for c in r['cases']),max_hold_shift=max(c['body_hold_observation']['max_center_shift_mm'] for c in r['cases'])))
