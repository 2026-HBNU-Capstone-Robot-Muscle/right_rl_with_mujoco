"""Promote the fully rechecked, current-hash model report; no devices."""
from horizontal_quarter_plan_20261027 import HorizontalQuarterPlan,REQUEST,PROOF,OUT
from horizontal_quarter_gate_20261027 import validate_review
plan=HorizontalQuarterPlan(REQUEST);verified=OUT/'review_final.json'
validate_review(plan,verified)
archive=OUT/'review_initial_before_type_validation.json'
if PROOF.exists() and not archive.exists():archive.write_bytes(PROOF.read_bytes())
PROOF.write_bytes(verified.read_bytes());validate_review(plan,PROOF)
print('Final v6 model review bound to unchanged current code; no device operations.')
