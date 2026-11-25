"""Adopt the passed offline software review for testing, not device execution."""
from horizontal_quarter_plan_20261027 import OUT,REQUEST,PROOF,HorizontalQuarterPlan
from horizontal_quarter_gate_20261027 import validate_review
source=OUT/'review_single_reply.json'
validate_review(HorizontalQuarterPlan(REQUEST),source)
PROOF.write_bytes(source.read_bytes())
print('Single-reply offline review validated for software tests; no devices opened.')
