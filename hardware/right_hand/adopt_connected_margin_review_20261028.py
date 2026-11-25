"""Adopt the fully screened reconnected start; no device access."""
import json
import calibrated_quarter_plan_20261028 as p
from calibrated_quarter_gate_20261028 import validate_review
source=p.ROOT/'right_hand_rotation_20261028/reconnected_170143_margin_v4_checked'
review=validate_review(p.HorizontalQuarterPlan(source/'request.json'),source/'review.json')
p.OUT.mkdir(exist_ok=True)
archive=p.OUT/'before_connected_adoption';archive.mkdir(exist_ok=True)
for current in (p.REQUEST,p.PROOF):
    if current.exists() and not (archive/current.name).exists():(archive/current.name).write_bytes(current.read_bytes())
for current in (p.REQUEST,p.PROOF):
    data=(source/current.name).read_bytes();pending=current.with_suffix(current.suffix+'.pending');pending.write_bytes(data);pending.replace(current)
review=validate_review(p.HorizontalQuarterPlan(p.REQUEST),p.PROOF)
print(json.dumps(dict(status='connected_start_and_EMPTY_margin_model_approved',plan_version=review['plan_version'],empty_cases=210,
    margin=review['empty_ring_pip_margin'],loaded_goals_unchanged=True,motor_writes=0)))
