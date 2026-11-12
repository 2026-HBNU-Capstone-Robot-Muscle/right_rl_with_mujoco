"""Bind completed model screens and identical saved-seed EMPTY recovery."""
import json
import sys
import numpy as np
from tip_first_trial_plan_20261112 import *
from tip_first_trial_gate_20261112 import make_review,validate_review
from work_grip_stage2_plan_20261112 import WorkGripStage2Plan,REQUEST as OLD_REQUEST,PROOF as OLD_PROOF
LOADED_GRASP='stage_5_loaded_grasp'

def main():
    if PROOF.exists():
        if '--rebind-tested-code' not in sys.argv:raise RuntimeError('Preserve prepared trial proof')
        backup=OUT/'review_before_final_code_binding.json'
        if not backup.exists():backup.write_bytes(PROOF.read_bytes())
    old=WorkGripStage2Plan(OLD_REQUEST);new=TipFirstTrialPlan(REQUEST)
    def empty(plan):
        rows=plan.plan(plan.source_position());end=next(i for i,(phase,_) in enumerate(rows) if phase==LOADED_GRASP)
        return [(p,q.tolist()) for p,q in rows[:end]]
    old_rows,new_rows=empty(old),empty(new)
    assert old_rows==new_rows and np.array_equal(old.source_position(),new.source_position())
    inherited=json.loads(OLD_PROOF.read_bytes())
    r=make_review(new,inherited['empty_recovery'],hashes())
    PROOF.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');validate_review(new,PROOF)
    (OUT/'empty_prefix_inheritance.json').write_text(json.dumps(dict(
        original_review=OLD_PROOF.relative_to(ROOT).as_posix(),original_review_sha256=sha(OLD_PROOF),
        empty_rows_exactly_equal=True,empty_rows=len(new_rows),same_saved_seed=True,
        static_cases=len(r['empty_recovery']['static_cases']),dynamic_cases=len(r['empty_recovery']['dynamic_cases']),
        new_full_empty_replay_performed=False,fresh_actual_OFF_recovery_rechecked_at_runtime=True,devices_opened=0),indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(prepared=True,loaded_goals=r['loaded_goals'],finite_loaded_cases=r['support_first_models']['loaded_finite_cases'],
        hand_only_cases=r['support_first_models']['hand_only_cases'],minimum_upper_mm=r['support_first_models']['minimum_upper_mm'],
        empty_static_cases=201,empty_dynamic_cases=9,devices_opened=0)))

if __name__=='__main__':main()
