"""Bind the single hook, unchanged safety flow and EMPTY hold stress cases."""
import ast,hashlib,json
from pathlib import Path
import numpy as np
import yaw_load_empty_gate_20261029 as base
from adaptive_yaw_empty_plan_20261029 import AdaptiveYawEmptyPlan,ROOT,OUT,REQUEST,PROOF,STRESS_PATH,hashes
from root_lag_pause_20261029 import POLICY
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen
def runner_review():
    original=ast.parse((ROOT/'horizontal_quarter_motion_runner_20261027.py').read_text(encoding='utf-8-sig'))
    hooked=ast.parse((ROOT/'root_lag_motion_runner_20261029.py').read_text(encoding='utf-8-sig'))
    removed=[]
    class RemoveHook(ast.NodeTransformer):
        def visit_Assign(self,node):
            if len(node.targets)==1 and isinstance(node.targets[0],ast.Name) and node.targets[0].id=='lag_hook':removed.append('binding');return None
            return self.generic_visit(node)
        def visit_If(self,node):
            if isinstance(node.test,ast.Compare) and isinstance(node.test.left,ast.Name) and node.test.left.id=='lag_hook':removed.append('call');return None
            return self.generic_visit(node)
    hooked=RemoveHook().visit(hooked)
    if sorted(removed)!=['binding','call'] or ast.dump(hooked,include_attributes=False)!=ast.dump(original,include_attributes=False):raise RuntimeError('Original motion health/goal/timeout/progress flow changed')
    return dict(only_addition='pre-write bounded wait hook',all_original_health_held_progress_timeout_cleanup_code_unchanged=True,
        original_sha256=hashlib.sha256((ROOT/'horizontal_quarter_motion_runner_20261027.py').read_bytes()).hexdigest())
def stress_review():
    path=OUT/'loaded_hand_dynamics/review.json';r=json.loads(path.read_bytes())
    if r.get('source_sha256')!=hashlib.sha256(STRESS_PATH.read_bytes()).hexdigest() or not r.get('all_passed') or not r.get('cube_disabled') or not r.get('finite_actuators'):raise RuntimeError('EMPTY fixed-goal stress rejected')
    if len(r['cases'])!=9 or {(c['book_front_mm'],c['gain_scale']) for c in r['cases']}!={(f,g) for f in (-15.,-10.,-5.) for g in (.5,1.,1.5)}:raise RuntimeError('EMPTY stress conditions incomplete')
    for c in r['cases']:
        if c['stop'] is not None or not c['passed'] or c['max_self_mm']>1e-6 or c['max_support_mm']>.15 or c['max_tracking_rad']>.18 or c['unapproved_range_excess_rad']>1e-6:raise RuntimeError('Original bounds failed in EMPTY hold stress')
    return dict(path=path.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),cases=9,
        hold_points=16,each_hold_s=1.2,added_hold_s=19.2,cube_disabled=True,arbitrary_adaptive_schedules_not_exhaustively_certified=True)
def make_review(p,empty,before):
    r=base.make_review(p,empty,base.hashes());after=hashes()
    if before!=after:raise RuntimeError('Source changed during bounded-wait review')
    r.update(status='EMPTY_bounded_catch_up_operator_review',code_hashes=after,root_lag_pause_policy=POLICY,
        original_runner_flow=runner_review(),fixed_goal_hold_stress=stress_review(),angular_targets_unchanged=True,
        force_and_health_stops_unchanged=True,hold_counts_toward300s=True,new_goals_or_resends_during_wait=0,
        physical_wait_controller_success_not_yet_verified=True)
    return r
def validate_review(p,path):
    r=json.loads(Path(path).read_bytes());expected=make_review(p,r['empty_recovery'],hashes())
    if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('EMPTY catch-up proof/source mismatch')
    return r
def run_screen(request,output,**kwargs):
    p=AdaptiveYawEmptyPlan(request);reference=AdaptiveYawEmptyPlan(REQUEST);r=validate_review(reference,PROOF)
    if np.array_equal(p.source_position(),reference.source_position()):
        r=dict(r);r.update(source_sha256=p.SOURCE_SHA,raw_off_ticks=p.raw_off_ticks.tolist(),review_center_reused=p.review_center_reused)
        Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return validate_review(p,output)
    process_screen(request,output,checker=ROOT/'screen_adaptive_yaw_empty_20261029.py',failure_details=lambda p:'adaptive EMPTY recovery rejected',**kwargs)
    return validate_review(p,output)
