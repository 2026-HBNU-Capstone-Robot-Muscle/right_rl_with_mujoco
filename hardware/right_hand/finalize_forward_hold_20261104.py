import json
from forward_hold_plan_20261104 import ForwardHoldPlan,REQUEST,PROOF,OUT,ROOT,hashes
from forward_hold_gate_20261104 import make_review,validate_review
def main():
    checks=json.loads((OUT/'software_verification.json').read_bytes())
    if not checks['passed']:raise RuntimeError('Software verification failed')
    if PROOF.exists():raise RuntimeError('Refuse to overwrite proof')
    p=ForwardHoldPlan(REQUEST);empty=json.loads((OUT/'empty_recovery_screen/review.json').read_bytes());r=make_review(p,empty,hashes());PROOF.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');validate_review(p,PROOF)
    print(json.dumps(dict(status='review_validated',whole_goals=len(p.plan(p.source_position())),seconds=len(p.plan(p.source_position()))*.075,cube_cases=27,hand_cases=9,empty_static_cases=len(empty['static_cases']),empty_dynamic_cases=len(empty['dynamic_cases']),new_joint_goal_vectors=0,automatic_open=False,fixed_hold_seconds=10.05,physical90_verified=False,physical_body_hold_verified=False,devices_opened=False)))
if __name__=='__main__':main()
