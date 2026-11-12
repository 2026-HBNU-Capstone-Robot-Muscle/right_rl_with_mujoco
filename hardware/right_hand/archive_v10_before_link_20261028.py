import json
from horizontal_quarter_plan_20261027 import ROOT,OUT,REQUEST,PROOF,HorizontalQuarterPlan,hashes
from horizontal_quarter_gate_20261027 import validate_review
validate_review(HorizontalQuarterPlan(REQUEST),PROOF)
dest=ROOT/'right_hand_rotation_20261028/v10_before_host_link_update';dest.mkdir(exist_ok=True)
for name in ('horizontal_quarter_plan_20261027.py','horizontal_quarter_gate_20261027.py','leap_horizontal_quarter_20261027.py'):
    if not (dest/name).exists():(dest/name).write_bytes((ROOT/name).read_bytes())
for name in ('request.json','review.json','verification.json','readiness.json'):
    if not (dest/name).exists():(dest/name).write_bytes((OUT/name).read_bytes())
if not (dest/'code_hashes.json').exists():(dest/'code_hashes.json').write_bytes(json.dumps(hashes(),indent=2).encode())
print('v10 sources and evidence preserved; no devices opened')
