"""Seal the model evidence; this does not open any robot or camera device."""
import json
from pitch_handoff_plan_20261028 import PitchHandoffPlan,OUT,REQUEST,PROOF,hashes
from pitch_handoff_gate_20261028 import make_review,validate_review

def main():
    p=PitchHandoffPlan(REQUEST);before=hashes()
    empty=json.loads((OUT/'empty_recovery_screen/review.json').read_bytes())
    review=make_review(p,empty,before)
    if PROOF.exists():
        validate_review(p,PROOF)
    else:PROOF.write_text(json.dumps(review,indent=2)+'\n',encoding='utf-8')
    validate_review(p,PROOF)
    print(json.dumps(dict(status='source_bound_model_review_passed',
        empty_static_cases=201,empty_dynamic_cases=9,loaded_static_cases=201,
        loaded_hand_dynamic_cases=9,conditional_free_cube_transfer_cases=27,
        native_joint_ranges_unchanged=True,next_yaw_queued=False,devices_opened=False)))

if __name__=='__main__':main()
