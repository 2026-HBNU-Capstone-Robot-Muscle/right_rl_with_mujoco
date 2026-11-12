"""Source-bound empty recovery/extended hold; no loaded-cube authorization."""
import ast,hashlib,json
from pathlib import Path
import numpy as np
from empty_hold_comm_plan_20261029 import ROOT,OUT,REQUEST,PROOF,HOLD_SECONDS,EmptyHoldCommPlan,hashes
from calibrated_quarter_gate_20261028 import validate_empty,separation_review,PRELOAD_POLICY,GOAL_POLICY,ROOT_POLICY,EPISODE_POLICY,CLOCK_POLICY
from calibrated_empty_ring_margin_20261028 import review as margin_review
from horizontal_thumb_book_numeric_20261027 import review as thumb_review
from retained_slow_yaw_gate_20261029 import fast_read_review,readonly_recovery_review,serial_link_review
from combined_quiet_reserve_20261029 import POLICY as RECEIVE_POLICY
from fast_health_window_20261029 import WINDOW_POLICY
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen
OFF_READ_SCOPES=((80,2),(82,2),(84,2),(102,2),(108,4),(112,4))
def read(p):return json.loads(Path(p).read_bytes())
def runner_review():
    a=ast.parse((ROOT/'horizontal_quarter_motion_runner_20261027.py').read_text(encoding='utf-8'))
    b=ast.parse((ROOT/'empty_hold_motion_runner_20261029.py').read_text(encoding='utf-8'))
    class RemoveExposure(ast.NodeTransformer):
        def visit_Assign(self,n):
            return None if any(isinstance(t,ast.Attribute) and t.attr=='empty_hold_guard' for t in n.targets) else self.generic_visit(n)
    original=next(n for n in a.body if isinstance(n,ast.FunctionDef) and n.name=='run')
    modified=RemoveExposure().visit(next(n for n in b.body if isinstance(n,ast.FunctionDef) and n.name=='run'))
    if ast.dump(original,include_attributes=False)!=ast.dump(modified,include_attributes=False):raise RuntimeError('Original motion/health runner changed beyond guard exposure')
    return dict(original_safety_flow_unchanged=True,original_held_joint_guard_reused=True,no_fake_cube_placement=True)
def make_review(plan,empty,before):
    validate_empty(plan,empty)
    if empty.get('fixed_final_hold_seconds')!=45.:raise RuntimeError('Full45s finite hold not screened')
    if empty.get('original_screen_source_sha256')!=hashlib.sha256((ROOT/'screen_calibrated_empty_margin_20261028.py').read_bytes()).hexdigest():raise RuntimeError('Original screen source differs')
    if before!=hashes():raise RuntimeError('Sources changed during review')
    rows=plan.plan(plan.source_position())
    return dict(status='empty_hold45s_comm_review',accepted_for_operator_trial=True,plan_version=plan.PLAN_VERSION,
        source_sha256=plan.SOURCE_SHA,start_ticks=plan.source_position().tolist(),raw_off_ticks=plan.raw_off_ticks.tolist(),
        review_center_reused=plan.review_center_reused,plan_sha256=plan.plan_digest(plan.source_position()),code_hashes=hashes(),
        empty_recovery=empty,initial_self_separation=separation_review(plan,empty),empty_ring_pip_margin=margin_review(plan),
        held_thumb_book_numerical_policy=thumb_review(plan,empty),runner_equivalence=runner_review(),fast_read_evidence=fast_read_review(),
        read_recovery_evidence=readonly_recovery_review(),verified_serial_link_policy=serial_link_review(),
        off_preload_confirmation_policy=dict(PRELOAD_POLICY),goal_readback_confirmation_policy=dict(GOAL_POLICY),
        empty_root_motion_policy=dict(ROOT_POLICY),read_resynchronization_policy=dict(RECEIVE_POLICY),
        read_recovery_episode_policy=dict(EPISODE_POLICY),precise_clock_policy=dict(CLOCK_POLICY),sync_receive_window_policy=dict(WINDOW_POLICY),
        OFF_setting_readback_scopes=[list(v) for v in OFF_READ_SCOPES],setting_write_retries=0,
        fixed_hold_seconds=45.,hold_sample_period_s=.075,sample_period_s=.075,goal_step_ticks=2,current_ma=450,
        active_motion_limit_s=300,nominal_seconds=len(rows)*.075+45.,cube_stays_OUT=True,no_loaded_path=True,
        hardware_health_thresholds_unchanged=True,hold_health_checks_unchanged=True,no_hold_goal_writes=True,assistant_motor_writes=0)
def validate_review(plan,path):
    r=read(path);expected=make_review(plan,r['empty_recovery'],hashes())
    if r!=expected:raise RuntimeError('Empty hold source/queue/bounds mismatch')
    return r
def run_screen(request,output,**kwargs):
    plan=EmptyHoldCommPlan(request);reference=EmptyHoldCommPlan(REQUEST);r=validate_review(reference,PROOF)
    if np.array_equal(plan.source_position(),reference.source_position()):
        r=make_review(plan,r['empty_recovery'],hashes());Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return r
    process_screen(request,output,checker=ROOT/'check_empty_hold_comm_20261029.py',failure_details=lambda p:'fresh empty recovery or45s hold rejected',**kwargs)
    return validate_review(plan,output)
