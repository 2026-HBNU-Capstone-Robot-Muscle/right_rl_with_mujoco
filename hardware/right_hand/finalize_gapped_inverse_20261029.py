"""Revalidate completed physics against the final admission code, no devices."""
import json
from gapped_inverse_plan_20261029 import GappedInversePlan,OUT,REQUEST,PROOF,hashes
from gapped_inverse_gate_20261029 import make_review,validate_review

def main():
    p=GappedInversePlan(REQUEST)
    empty=json.loads((OUT/'empty_recovery_screen/review.json').read_bytes())
    before=hashes();r=make_review(p,empty,before)
    # Only provenance validation was strengthened after this prefix computation;
    # the current gate checks the exact prefix digest and every original bound.
    PROOF.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
    validate_review(p,PROOF)
    print(json.dumps(dict(status=r['status'],accepted=r['accepted_for_operator_trial'],
        total_goals=len(p.plan(p.source_position())),inverse_model=r['inverse_model'],
        new_geometry_cases=r['additional_hand_path']['new_geometry_cases'],
        prefix_static=len(empty['static_cases']),prefix_dynamic=len(empty['dynamic_cases']),
        assistant_motor_writes=0)))
if __name__=='__main__':main()
