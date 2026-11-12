"""Copy the completed model review to the entrypoint's canonical path."""
from adaptive_yaw_empty_plan_20261029 import OUT,PROOF,REQUEST,AdaptiveYawEmptyPlan
from adaptive_yaw_empty_gate_20261029 import validate_review
source=OUT/'operator_review.json'
validate_review(AdaptiveYawEmptyPlan(REQUEST),source)
if PROOF.exists():raise RuntimeError('Refuse to replace an existing proof')
PROOF.write_bytes(source.read_bytes())
review=validate_review(AdaptiveYawEmptyPlan(REQUEST),PROOF)
print(review['status'])
