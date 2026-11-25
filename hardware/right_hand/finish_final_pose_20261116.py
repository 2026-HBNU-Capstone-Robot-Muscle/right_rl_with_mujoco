"""Rebind unchanged model observations after source review and summarize checks."""
import json
from final_grip_pose_plan_20261116 import ROOT,OUT,REQUEST,PROOF,FinalGripPosePlan,hashes
from final_grip_pose_gate_20261116 import make_review,validate_review
p=FinalGripPosePlan(REQUEST)
empty=json.loads((OUT/'empty_recovery_screen/review.json').read_bytes())
full=json.loads((OUT/'full_pose_screen/review.json').read_bytes())
r=make_review(p,empty,full,hashes())
PROOF.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
validate_review(p,PROOF)
print(json.dumps(dict(status=r['status'],goals=len(p.plan(p.source_position())),seconds=len(p.plan(p.source_position()))*.075,
    static_cases=len(full['static_cases']),dynamic_cases=len(full['dynamic_cases']),devices_opened=False)))
