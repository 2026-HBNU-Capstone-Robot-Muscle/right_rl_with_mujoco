"""Restore the operator proof after a clean current-connection observation."""
import json
from horizontal_quarter_plan_20261027 import ROOT,OUT,REQUEST,PROOF,HorizontalQuarterPlan
from horizontal_quarter_gate_20261027 import validate_review
folder=OUT/'after_usb_swap_044716'
r=json.loads((ROOT/'usb_swap_readonly_20261028_044716_809381/result.json').read_bytes())
assert r['status']=='swapped_usb_readonly_observation_complete' and not r['errors'] and r['blank_frames']==0
assert r['camera_observed_duration_s']>=55 and r['reads']>=700
validate_review(HorizontalQuarterPlan(folder/'request.json'),folder/'review.json')
archive=OUT/'before_usb_swap_readiness';archive.mkdir(exist_ok=True)
for name in ('request.json','review.json','verification.json','readiness.json'):
    if (OUT/name).exists() and not (archive/name).exists():(archive/name).write_bytes((OUT/name).read_bytes())
REQUEST.write_bytes((folder/'request.json').read_bytes());PROOF.write_bytes((folder/'review.json').read_bytes())
validate_review(HorizontalQuarterPlan(REQUEST),PROOF)
print('Fresh swapped-port source/proof adopted. Motion remains user-started only; no devices opened.')
