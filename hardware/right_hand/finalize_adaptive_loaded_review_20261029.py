"""Join the independently completed current recovery and loaded pause cases."""
import json
from adaptive_yaw_loaded_plan_20261029 import AdaptiveYawLoadedPlan,OUT,REQUEST,PROOF
from adaptive_yaw_loaded_gate_20261029 import make_review,validate_review
if PROOF.exists():raise RuntimeError('Refuse to overwrite existing proof')
empty=json.loads((OUT/'empty_recovery_screen/review.json').read_bytes())
before=json.loads((OUT/'empty_recovery_screen/control_source_hashes.json').read_bytes())
r=make_review(AdaptiveYawLoadedPlan(REQUEST),empty,before)
PROOF.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
validate_review(AdaptiveYawLoadedPlan(REQUEST),PROOF)
print(r['status'])
