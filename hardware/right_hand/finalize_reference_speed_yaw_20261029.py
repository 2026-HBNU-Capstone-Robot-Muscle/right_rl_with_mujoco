import json,sys
from reference_speed_yaw_plan_20261029 import ReferenceSpeedYawPlan,OUT,REQUEST,PROOF,hashes
from reference_speed_yaw_gate_20261029 import make_review,validate_review,completed_reference
if '--reference-only' in sys.argv:
    print(json.dumps(completed_reference()));raise SystemExit()
p=ReferenceSpeedYawPlan(REQUEST);empty=json.loads((OUT/'empty_recovery_screen/review.json').read_bytes())
r=make_review(p,empty,hashes());PROOF.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');validate_review(p,PROOF)
print(json.dumps(dict(status='offline_review_complete',static_cases=len(empty['static_cases']),dynamic_cases=len(empty['dynamic_cases']),
    loaded_model_cases=r['loaded_model_cases'],goals=len(p.plan(p.source_position())),nominal_motion_seconds=r['nominal_motion_seconds'],devices_opened=False)))
