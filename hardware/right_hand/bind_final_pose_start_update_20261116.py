"""Bind only a complete, passing updated proof. Never overrides failed cases."""
import json
from final_grip_pose_plan_20261116 import OUT,REQUEST,PROOF,FinalGripPosePlan,hashes
from final_grip_pose_gate_20261116 import make_review,validate_review
folder=OUT/'range_fix_verified'
p=FinalGripPosePlan(REQUEST)
empty=json.loads((folder/'empty_recovery_screen/review.json').read_bytes())
full=json.loads((folder/'full_pose_screen/review.json').read_bytes())
r=make_review(p,empty,full,hashes())
PROOF.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
validate_review(p,PROOF)
print(json.dumps(dict(status=r['status'],goals=len(p.plan(p.source_position())),seconds=len(p.plan(p.source_position()))*.075,
    static_passed=sum(v['passed'] for v in full['static_cases']),dynamic_passed=sum(v['passed'] for v in full['dynamic_cases']),devices_opened=False)))
