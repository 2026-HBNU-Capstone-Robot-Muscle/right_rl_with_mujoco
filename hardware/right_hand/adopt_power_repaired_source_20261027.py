"""Adopt the fresh verified OFF source after user-confirmed V- repair."""
import json
from horizontal_quarter_plan_20261027 import ROOT,OUT,REQUEST,PROOF,HorizontalQuarterPlan
from horizontal_quarter_gate_20261027 import validate_review
new=ROOT/'right_hand_rotation_20261027/power_repaired_214344'
source_request=new/'request.json';source_proof=new/'review.json'
review=validate_review(HorizontalQuarterPlan(source_request),source_proof)
archive=OUT/'source_archive_204724';archive.mkdir(exist_ok=True)
for source,name in ((REQUEST,'request.json'),(PROOF,'review.json'),(OUT/'verification.json','verification.json')):
    target=archive/name
    if source.exists() and not target.exists():target.write_bytes(source.read_bytes())
REQUEST.write_bytes(source_request.read_bytes());PROOF.write_bytes(source_proof.read_bytes())
validate_review(HorizontalQuarterPlan(REQUEST),PROOF)
print(json.dumps(dict(status='fresh_power_repaired_source_adopted',model_cases=210,
    initial_self_exception_enabled=review['initial_self_separation']['enabled'],
    held_thumb_numerical_allowance_enabled=review['held_thumb_book_numerical_policy']['enabled'],
    assistant_motor_writes=0)))
