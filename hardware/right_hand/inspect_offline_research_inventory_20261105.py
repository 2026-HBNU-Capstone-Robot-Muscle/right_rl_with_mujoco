"""Compact local report inventory; no simulation and no devices."""
import json
from pathlib import Path
O=Path(__file__).resolve().parent/'right_hand_rotation_20261105/canonical_body_grip'
for pattern in ('face_hold_hypotheses/review.json','cube_symmetry_review.json','model_discrepancy/photo_angle_sensitivity.json','support_release_search/*/review.json','grasp_timing_search/*/review.json','workspace_transition/*/review.json','workspace_lift_yaw/*/review.json','rolling_feedback/*/review.json','wide_initial_yaw/*/review.json'):
    for p in sorted(O.glob(pattern)):
        d=json.loads(p.read_bytes());cases=d.get('cases',[])
        print(json.dumps(dict(path=str(p.relative_to(O)),keys=list(d),cases=len(cases) if isinstance(cases,list) else str(type(cases)),first_case_keys=list(cases[0]) if isinstance(cases,list) and cases else None),ensure_ascii=True))
