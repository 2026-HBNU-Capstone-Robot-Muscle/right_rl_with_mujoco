"""Reuse only the exactly identical, independently screened EMPTY prefix."""
import hashlib,json
from reverse_yaw_empty_plan_20261029 import ReverseYawEmptyPlan,ROOT,REQUEST,PROOF,hashes
from reverse_yaw_empty_gate_20261029 import make_review,validate_review
from calibrated_quarter_plan_20261028 import prefix_digest
source=ROOT/'right_hand_rotation_20261029/observed_loaded_160451/next_repeat_recovery/review.json'
prior=json.loads(source.read_bytes());p=ReverseYawEmptyPlan(REQUEST);empty=prior['empty_recovery']
assert empty['start_ticks']==p.source_position().tolist() and empty['prefix_sha256']==prefix_digest(p)
if PROOF.exists():raise RuntimeError('Refuse to overwrite reverse proof')
r=make_review(p,empty,hashes());r['identical_empty_recovery_reused_from']=source.relative_to(ROOT).as_posix()
r['identical_empty_recovery_source_sha256']=hashlib.sha256(source.read_bytes()).hexdigest()
PROOF.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');validate_review(p,PROOF)
print(r['status'])
