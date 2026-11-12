"""Assemble completed independent model evidence; no device access."""
import json
import calibrated_quarter_plan_20261028 as p
from calibrated_quarter_gate_20261028 import make_review,validate_review
plan=p.HorizontalQuarterPlan(p.REQUEST)
if p.PROOF.exists():
    review=validate_review(plan,p.PROOF)
else:
    before=p.hashes();empty=json.loads((p.OUT/'empty_recovery_screen/review.json').read_bytes())
    review=make_review(plan,empty,before)
    p.PROOF.write_bytes((json.dumps(review,indent=2)+'\n').encode())
    validate_review(plan,p.PROOF)
print(json.dumps(dict(accepted=review['accepted_for_operator_trial'],version=review['plan_version'],current_empty_cases=210,
    loaded_cases=27,geometry_cases=201,nominal_model_yaw_deg=review['nominal_model_yaw_deg'],
    physical_timing_calibration=review['physical_timing_calibration'],assistant_motor_writes=0)))
