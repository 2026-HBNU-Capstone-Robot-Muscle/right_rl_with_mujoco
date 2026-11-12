import json
from empty_hold_comm_plan_20261029 import EmptyHoldCommPlan,OUT,REQUEST,PROOF,hashes
from empty_hold_comm_gate_20261029 import make_review,validate_review
p=EmptyHoldCommPlan(REQUEST);empty=json.loads((OUT/'current_empty_screen/review.json').read_bytes())
r=make_review(p,empty,hashes());PROOF.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');validate_review(p,PROOF)
print(json.dumps(dict(status='empty_hold_comm_ready',prefix_goals=len(p.plan(p.source_position())),hold_seconds=45,nominal_seconds=r['nominal_seconds'],model_cases=210,assistant_motor_writes=0)))
