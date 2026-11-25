"""Preserve the observed working v9 sources and proof before a narrow update."""
import json,hashlib
from horizontal_quarter_plan_20261027 import ROOT,OUT,REQUEST,PROOF,hashes
from horizontal_quarter_gate_20261027 import validate_review
from horizontal_quarter_plan_20261027 import HorizontalQuarterPlan
validate_review(HorizontalQuarterPlan(REQUEST),PROOF)
dest=ROOT/'right_hand_rotation_20261028/v9_before_confirmation_burst_fix';dest.mkdir(exist_ok=True)
names=['horizontal_quarter_plan_20261027.py','horizontal_quarter_gate_20261027.py',
    'horizontal_quarter_health_20261027.py','horizontal_receive_resync_20261027.py',
    'horizontal_sync_receive_window_20261028.py','leap_horizontal_quarter_20261027.py']
for name in names:
    target=dest/name
    if not target.exists():target.write_bytes((ROOT/name).read_bytes())
for name in ('request.json','review.json','readiness.json','verification.json','operator_guide.md'):
    if (OUT/name).exists() and not (dest/name).exists():(dest/name).write_bytes((OUT/name).read_bytes())
if not (dest/'code_hashes.json').exists():(dest/'code_hashes.json').write_bytes(json.dumps(hashes(),indent=2).encode())
print('v9 controller and evidence archived; no devices opened')
