"""Finalize only the tested placement-photo trial, never a loaded rotation trial."""
import json,shutil
from pathlib import Path
from placement_alignment_plan_20261030 import ROOT,OUT,REQUEST,PROOF,PlacementAlignmentPlan,hashes,sha
from placement_alignment_gate_20261030 import make_review,validate_review
def main():
 tests=json.loads((OUT/'software_admission_result.json').read_bytes());camera=json.loads((OUT/'camera_probe/result.json').read_bytes())
 assert tests['status']=='passed' and tests['tests_run']==22 and not tests['failures'] and not tests['errors'] and tests['controller_hashes']==hashes()
 assert camera['status']=='camera_only_comparison_checked' and camera['motor_port_constructions']==0 and camera['raw_shape']==[720,1280,3] and camera['camera_recording']['status']=='complete'
 p=PlacementAlignmentPlan(REQUEST);empty=json.loads((OUT/'empty_recovery_screen/review.json').read_bytes());report=make_review(p,empty,hashes())
 archive=OUT/'review_before_final_software_scope_checks.json'
 if PROOF.exists() and not archive.exists():shutil.copyfile(PROOF,archive)
 PROOF.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');validate_review(p,PROOF)
 meta=dict(status='placement_alignment_photo_trial_ready',scope='EMPTY preparation only, then fixed-goal photo/removal; projected yellow guide is display only',model_static_cases=201,model_dynamic_cases=9,software_tests=22,source_readonly=json.loads(REQUEST.read_bytes())['source'],camera_only_probe=(OUT/'camera_probe/result.json').relative_to(ROOT).as_posix(),proof_sha256=sha(PROOF),motor_current_or_PID_change_from_previous_trials=False,goal_step_ticks=2,post_placement_position_writes=0,grasp_or_rotation_trial_ready=False,operator_command='& "'+str(ROOT/'leap_placement_alignment.cmd')+'"',user_action_required=True,assistant_motor_writes=0)
 (OUT/'handoff.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:meta[k] for k in ('status','model_static_cases','model_dynamic_cases','software_tests','post_placement_position_writes','grasp_or_rotation_trial_ready','assistant_motor_writes')}))
if __name__=='__main__':main()
