"""Retain existing prefix/model/health checks; teaching starts only after OFF."""
import json
from pathlib import Path
import numpy as np
import natural_cradle_grasp_gate_20261030 as original
from natural_cradle_grasp_plan_20261030 import hashes as original_hashes
from manual_grasp_plan_20261106 import R,REQUEST,PROOF,ManualGraspPlan,hashes,validate_candidate
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen
from verified_off_restore_context_20261105 import POLICY as RESTORE_POLICY
def make_review(plan,empty,before):
    z=validate_candidate();r=original.make_review(plan,empty,original_hashes());after=hashes()
    if before!=after:raise RuntimeError('Teaching sources changed during review')
    r['reviewed_full_source_goal_count']=r.pop('grasp_only_loaded_goals')
    r.update(status='supervised_executed_grasp_prefix_then_OFF_joint_teaching',scope='Same executed grasp prefix; support/SPACE OFF; manual joint changes and photo/encoder READS only afterward',code_hashes=after,loaded_goals=len(z['goals']),automatic_open=False,automatic_inverse=False,automatic_reenable_after_teaching=False,manual_joint_motion_requires_verified_torque_off=True,new_joint_goal_vectors=0,rotation_targets=0,source_model_scope='Existing27 full-path safety cases include this unchanged grasp/hold prefix; corrected pose and OFF teaching are not certified grips',post_off_restore_policy=dict(RESTORE_POLICY),manual_pose_not_a_new_motion_admission=True)
    return r
def validate_review(plan,path):
    r=json.loads(Path(path).read_bytes());expected=make_review(plan,r['empty_recovery'],hashes())
    if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('Teaching source/path/health scope mismatch')
    return r
def run_screen(request,output,**kwargs):
    p=ManualGraspPlan(request);reference=ManualGraspPlan(REQUEST);r=validate_review(reference,PROOF)
    if np.array_equal(p.source_position(),reference.source_position()):
        result=make_review(p,r['empty_recovery'],hashes());Path(output).write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');return validate_review(p,output)
    process_screen(request,output,checker=R/'screen_manual_grasp_current_20261106.py',failure_details=lambda p:'fresh EMPTY recovery rejected',**kwargs)
    return validate_review(p,output)
