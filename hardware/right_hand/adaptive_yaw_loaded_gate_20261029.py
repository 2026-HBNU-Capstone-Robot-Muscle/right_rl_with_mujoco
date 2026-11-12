"""Bind unchanged physical targets and health flow plus explicit pause stress."""
import hashlib,json
from pathlib import Path
import numpy as np
import four_reply_yaw_gate_20261029 as base
from adaptive_yaw_loaded_plan_20261029 import AdaptiveYawLoadedPlan,ROOT,OUT,REQUEST,PROOF,STRESS_PATH,EMPTY_EVIDENCE,LOADED_PATH,hashes
from root_lag_pause_20261029 import POLICY
from adaptive_yaw_empty_gate_20261029 import runner_review
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen
def empty_evidence():
    r=json.loads(EMPTY_EVIDENCE.read_bytes());source=ROOT/r['source']
    if hashlib.sha256(source.read_bytes()).hexdigest()!=r['source_sha256'] or not r['completed'] or r['cube_present'] or not r['all_torque_off_verified'] or not r['settings_restored'] or r['yaw_samples']!=320 or not r['all_actual_goal_writes_match_reviewed_queue']:raise RuntimeError('Actual EMPTY run evidence rejected')
    return r
def stress_review():
    path=OUT/'loaded_pause_refined_27/review.json';r=json.loads(path.read_bytes());z=np.load(STRESS_PATH);baseline=np.load(LOADED_PATH)
    if r.get('simulation_timestep_s')!=.001 or r.get('settling_seconds')!=2 or not r.get('physics_acceptance_bounds_unchanged'):raise RuntimeError('Refined grid timing mismatch')
    blocks=r.get('blocks',[])
    if len(blocks)!=3:raise RuntimeError('Missing refined grid blocks')
    for block in blocks:
        if hashlib.sha256((ROOT/block['path']).read_bytes()).hexdigest()!=block['sha256'] or not block['all_passed']:raise RuntimeError('Refined block source mismatch')
    convergence_path=OUT/'numerical_convergence.json';convergence=json.loads(convergence_path.read_bytes())
    if not convergence.get('all_passed') or len(convergence.get('cases',[]))!=4:raise RuntimeError('Step refinement checks incomplete')
    if {(c['simulation_timestep_s'],c['cases'][0]['gain_scale'],c['cases'][0]['friction']) for c in convergence['cases']}!={(dt,g,mu) for dt in (.001,.0005) for g,mu in ((1.5,.2),(1.,.3))}:raise RuntimeError('Step refinement conditions incomplete')
    for c in convergence['cases']:
        if c['candidate_sha256']!=hashlib.sha256(STRESS_PATH.read_bytes()).hexdigest() or not c['physics_acceptance_bounds_unchanged'] or not c['all_passed']:raise RuntimeError('Step refinement rejected')
    if not np.array_equal(z['goals'][~z['fixed_hold_rows']],baseline['goals'][baseline['phases']=='yaw']) or np.count_nonzero(z['fixed_hold_rows'])!=256:raise RuntimeError('Fixed-goal stress changed angular route')
    if float(z['added_hold_seconds'])!=19.2 or float(z['each_hold_seconds'])!=POLICY['maximum_single_wait_s'] or float(z['added_hold_seconds'])>POLICY['maximum_total_wait_s']:raise RuntimeError('Pause stress scope mismatch')
    if r['candidate_sha256']!=hashlib.sha256(STRESS_PATH.read_bytes()).hexdigest() or r['loaded_path_sha256']!=hashlib.sha256((path.parent/'loaded_path.npz').read_bytes()).hexdigest():raise RuntimeError('Pause stress source mismatch')
    if r.get('all_passed') is not True or not r.get('finite_stiffness_contact_forces_preserved') or r['physical_yaw_verified'] or r['hardware_execution_allowed']:raise RuntimeError('Loaded pause stress failed')
    cases=r['cases'];expected={(f,g,mu) for f in (-15.,-10.,-5.) for g in (.5,1.,1.5) for mu in (.2,.3,.5)}
    if len(cases)!=27 or {(c['book_front_mm'],c['gain_scale'],c['friction']) for c in cases}!=expected:raise RuntimeError('Loaded pause stress conditions incomplete')
    from calibrated_quarter_gate_20261028 import bounded
    for c in cases:
        if not c['passed'] or c['stop'] is not None or c['external_cube_contact'] or not c['final_cube_supported_by_hand'] or not bounded(c,{'max_self_mm':1e-6,'max_support_mm':.15,'max_cube_mm':1.,'max_tracking_rad':.18,'unapproved_range_excess_rad':1e-6,'center_translation_mm':80.,'longest_no_hand_contact_seconds':.25}):raise RuntimeError('Loaded pause stress violated original criteria')
    return dict(path=path.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),cases=27,
        hold_points=z['hold_after_original_targets'].tolist(),each_hold_seconds=1.2,total_extra_seconds=19.2,
        angle_range_deg=[min(c['yaw_before_release_deg'] for c in cases),max(c['yaw_before_release_deg'] for c in cases)],
        simulation_timestep_s=.001,numerical_refinement_review=convergence_path.relative_to(ROOT).as_posix(),
        numerical_refinement_sha256=hashlib.sha256(convergence_path.read_bytes()).hexdigest(),
        coarse2ms_grid_rejected=True,contact_depth_not_fully_numerically_converged=True,
        arbitrary_adaptive_schedules_not_exhaustively_certified=True,physical_rotation_not_yet_verified=True)
def make_review(p,empty,before):
    r=base.make_review(p,empty,base.hashes());after=hashes()
    if before!=after:raise RuntimeError('Source changed during loaded catch-up review')
    r.update(status='supervised_loaded_bounded_catch_up_review',code_hashes=after,root_lag_pause_policy=dict(POLICY),
        actual_EMPTY_result=empty_evidence(),original_runner_flow=runner_review(),loaded_fixed_goal_pause_stress=stress_review(),
        original_angular_targets_unchanged=True,yaw_goal_count=638,hardware_wait_activation_still_unverified=True,
        wait_included_in_original300s=True,new_goals_or_resends_during_wait=0)
    return r
def validate_review(p,path):
    r=json.loads(Path(path).read_bytes());expected=make_review(p,r['empty_recovery'],hashes())
    if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('Loaded catch-up proof/source mismatch')
    return r
def run_screen(request,output,**kwargs):
    p=AdaptiveYawLoadedPlan(request);reference=AdaptiveYawLoadedPlan(REQUEST);r=validate_review(reference,PROOF)
    if np.array_equal(p.source_position(),reference.source_position()):
        r=dict(r);r.update(source_sha256=p.SOURCE_SHA,raw_off_ticks=p.raw_off_ticks.tolist(),review_center_reused=p.review_center_reused)
        Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return validate_review(p,output)
    process_screen(request,output,checker=ROOT/'screen_adaptive_yaw_loaded_20261029.py',failure_details=lambda p:'loaded catch-up current recovery rejected',**kwargs)
    return validate_review(p,output)
