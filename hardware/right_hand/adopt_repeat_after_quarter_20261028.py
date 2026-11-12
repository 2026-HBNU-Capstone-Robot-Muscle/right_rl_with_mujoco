"""Keep the working controller; replace only its screened OFF reference."""
from horizontal_quarter_plan_20261027 import ROOT,OUT,REQUEST,PROOF,HorizontalQuarterPlan
from horizontal_quarter_gate_20261027 import validate_review
source=ROOT/'right_hand_rotation_20261028/repeat_after_quarter_012305'
validate_review(HorizontalQuarterPlan(REQUEST),PROOF)
validate_review(HorizontalQuarterPlan(source/'request.json'),source/'review.json')
archive=OUT/'source_archive_first_success_010146';archive.mkdir(exist_ok=True)
for path,name in ((REQUEST,'request.json'),(PROOF,'review.json'),(OUT/'verification.json','verification.json')):
    target=archive/name
    if path.exists() and not target.exists():target.write_bytes(path.read_bytes())
REQUEST.write_bytes((source/'request.json').read_bytes());PROOF.write_bytes((source/'review.json').read_bytes())
validate_review(HorizontalQuarterPlan(REQUEST),PROOF)
print('REPEAT SOURCE VERIFIED. Same controller and loaded goals. No devices opened.')
