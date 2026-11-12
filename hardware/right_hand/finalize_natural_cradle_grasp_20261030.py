"""Bind full safety evidence and honest nominal-only grasp-task evidence."""
import json
from natural_cradle_grasp_plan_20261030 import OUT,REQUEST,PROOF,NaturalCradleGraspPlan,hashes
from natural_cradle_grasp_gate_20261030 import make_review,validate_review
def main():
 p=NaturalCradleGraspPlan(REQUEST);empty=json.loads((OUT/'empty_recovery_screen/review.json').read_bytes());r=make_review(p,empty,hashes());PROOF.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');validate_review(p,PROOF);print(json.dumps(dict(status=r['status'],scope=r['scope'],cube_safety_cases=r['natural_cradle_models']['cube_safety_cases'],grasp_task_passed_cases=r['natural_cradle_models']['grasp_task_passed_cases'],nonnominal_task_failures=len(r['natural_cradle_models']['nonnominal_grasp_task_failures']),rotation_targets=0,motor_writes=0)))
if __name__=='__main__':main()
