"""Source-bound review for one supervised, continuous two-axis experiment."""
import hashlib,json
from pathlib import Path
import numpy as np
from pitch_slow_yaw_plan_20261029 import PitchSlowYawPlan,ROOT,OUT,REQUEST,PROOF,LOADED_PATH,loaded_rows,hashes
from pitch_handoff_gate_20261028 import validate_loaded as validate_pitch_evidence
from calibrated_quarter_gate_20261028 import (validate_empty,separation_review,prefix_digest,bounded,
    PRELOAD_POLICY,GOAL_POLICY,ROOT_POLICY,RECEIVE_POLICY,WINDOW_POLICY,EPISODE_POLICY,CLOCK_POLICY)
from calibrated_quarter_plan_20261028 import STATIC_REVIEW
from retained_red_yaw_plan_20261029 import ORIGINAL as ORIGINAL_YAW
from calibrated_empty_ring_margin_20261028 import review as margin_review
from horizontal_thumb_book_numeric_20261027 import review as thumb_review
from horizontal_serial_link_20261028 import metadata as link_review
from slow_thumb_seam_resolution_20261026 import metadata as seam_metadata
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from book_empty_preparation_plan import FRONTS,offsets
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen

def read(path):return json.loads(Path(path).read_bytes())
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def validate_loaded():
    historical=validate_pitch_evidence()
    rows=loaded_rows();hand=read(OUT/'loaded_hand_dynamics/review.json')
    if hand.get('source_sha256')!=sha(LOADED_PATH) or hand.get('all_passed') is not True or hand.get('cube_disabled') is not True:raise RuntimeError('Combined finite hand review missing/rejected')
    if hand.get('existing_palm7_seam_policy')!=seam_metadata():raise RuntimeError('Existing seam scope changed')
    if len(hand['cases'])!=9 or {(r['book_front_mm'],r['gain_scale']) for r in hand['cases']}!={(f,g) for f in FRONTS for g in (.5,1.,1.5)}:raise RuntimeError('Combined hand conditions incomplete')
    for r in hand['cases']:
        if r.get('passed') is not True or r.get('stop') is not None or not bounded(r,{'max_self_mm':1e-6,'max_support_mm':.15,'max_tracking_rad':.18,'unapproved_range_excess_rad':1e-6,'existing_palm7_seam_discarded_max_mm':.01}):raise RuntimeError('Combined hand dynamics bound failed')
    # No new angular target is introduced. This review is the union of the
    # original pitch/transfer geometry and the original yaw geometry. The
    # slower yaw changes timing by scheduled duplicate targets only.
    geometry=read(STATIC_REVIEW)
    if geometry.get('all_passed') is not True or geometry.get('all_offsets_requested') is not True:raise RuntimeError('Original yaw geometry evidence missing')
    if len(geometry['cases'])!=201 or geometry.get('source')!=ORIGINAL_YAW.relative_to(ROOT).as_posix():raise RuntimeError('Yaw geometry cases missing')
    if {(v['book_front_mm'],tuple(v['offset_ticks'])) for v in geometry['cases']}!={(f,tuple(o)) for f in FRONTS for o in offsets()}:raise RuntimeError('Yaw geometry neighborhood incomplete')
    if any(v.get('passed') is not True or not bounded(v,{'max_self_mm':1e-6,'max_support_mm':.15}) for v in geometry['cases']):raise RuntimeError('Yaw geometry bounds failed')
    cube=read(OUT/'loaded_cube_evidence.json')
    if cube.get('accepted_for_supervised_combined_trial') is not True or cube.get('loaded_path_sha256')!=sha(LOADED_PATH):raise RuntimeError('Continuous free-cube review not ready')
    if cube.get('case_count')!=27 or cube.get('nominal_case_health_passed') is not True:raise RuntimeError('Continuous cube nominal conditions missing/failed')
    if cube.get('wide_parameter_robustness_verified') is not False or cube.get('supervised_experiment_only') is not True:raise RuntimeError('Experimental validation scope changed')
    expected={(f,g,mu) for f in FRONTS for g in (.5,1.,1.5) for mu in (.2,.3,.5)}
    if {(v['book_front_mm'],v['gain_scale'],v['friction']) for v in cube['cases']}!=expected:raise RuntimeError('Continuous cube grid mismatch')
    for r in cube['cases']:
        # The broad uncalibrated parameter grid is diagnostic, and its failures
        # must remain visible. Only nominal cases are admitted as loaded evidence
        # for this supervised experiment. This is not a robust-controller release.
        if (r['gain_scale'],r['friction'])!=(1.,.3):continue
        if r.get('motion_health_passed') is not True or r.get('stop') is not None or r.get('external_cube_contact') is not False or not bounded(r,{
            'max_self_mm':1e-6,'max_support_mm':.15,'max_cube_overlap_mm':1.,'max_tracking_rad':.18,
            'native_range_excess_rad':1e-6,'max_no_hand_contact_seconds':.25}):raise RuntimeError('Continuous cube bounds failed')
    if len(cube.get('nominal_pose_sensitivity_cases',[]))!=9 or not all(r.get('motion_health_passed') is True for r in cube['nominal_pose_sensitivity_cases']):raise RuntimeError('Nominal retained-pose neighborhood rejected')
    for source in cube['bound_sources']:
        if sha(ROOT/source['path'])!=source['sha256']:raise RuntimeError('Continuous cube source changed')
    return dict(historical_pitch=historical,loaded_targets=len(rows),combined_hand_cases=9,continuous_free_cube_cases=27,
        geometry_source_sets=2,loaded_hand_review_sha256=sha(OUT/'loaded_hand_dynamics/review.json'),
        loaded_cube_review_sha256=sha(OUT/'loaded_cube_evidence.json'),yaw_geometry_sha256=sha(STATIC_REVIEW),
        nominal_loaded_conditions_passed=True,all27_parameter_conditions_passed=False,nominal_retained_pose_cases=9,
        wide_parameter_failures_preserved=True,supervised_experiment_only=True,
        cube_pose_initialized_once=True,cube_pose_forced_during_rollout=False,initial_cube_pose_is_conditional=True,
        robust_exact90_degrees_not_certified=True,combined_physical_success_not_yet_verified=True)

def make_review(plan,empty,before):
    validate_empty(plan,empty);loaded=validate_loaded();after=hashes()
    if before!=after:raise RuntimeError('Sources changed during review')
    rows=plan.plan(plan.source_position())
    return dict(status='supervised_continuous_pitch_then_slower_yaw_review',accepted_for_operator_trial=True,
        plan_version=plan.PLAN_VERSION,source_sha256=plan.SOURCE_SHA,start_ticks=plan.source_position().tolist(),
        raw_off_ticks=plan.raw_off_ticks.tolist(),review_center_reused=plan.review_center_reused,reviewed_start_radius_ticks=4,
        plan_sha256=plan.plan_digest(plan.source_position()),last_goal_ticks=rows[-1][1].tolist(),mapping=list(SIM_TO_MOTOR_IDS),
        code_hashes=after,code_unchanged=True,empty_recovery=empty,loaded_evidence=loaded,
        initial_self_separation=separation_review(plan,empty),empty_ring_pip_margin=margin_review(plan),held_thumb_book_numerical_policy=thumb_review(plan,empty),
        off_preload_confirmation_policy=dict(PRELOAD_POLICY),goal_readback_confirmation_policy=dict(GOAL_POLICY),
        empty_root_motion_policy=dict(ROOT_POLICY),read_resynchronization_policy=dict(RECEIVE_POLICY),
        sync_receive_window_policy=dict(WINDOW_POLICY),read_recovery_episode_policy=dict(EPISODE_POLICY),
        precise_clock_policy=dict(CLOCK_POLICY),verified_serial_link_policy=link_review(),
        sample_period_s=.075,distinct_yaw_goal_period_s=.15,goal_step_ticks=2,current_ma=450,
        active_motion_limit_s=480,old_program_active_limit_s=300,old_programs_unchanged=True,
        nominal_motion_seconds=len(rows)*.075,maximum_aggregate_thumb_displacement_ticks=1568,
        native_joint_ranges_unchanged=True,hardware_health_thresholds_unchanged=True,no_live_policy=True,
        user_place_once_before_pitch=True,physical_combined_rotation_verified=False,assistant_motor_writes=0)

def validate_review(plan,path):
    r=read(path);expected=make_review(plan,r['empty_recovery'],hashes())
    if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('Combined source, queue, bounds, or review mismatch')
    return r

def run_screen(request,output,**kwargs):
    plan=PitchSlowYawPlan(request);reference=PitchSlowYawPlan(REQUEST);reviewed=validate_review(reference,PROOF)
    if np.array_equal(plan.source_position(),reference.source_position()):
        r=dict(reviewed);r.update(source_sha256=plan.SOURCE_SHA,raw_off_ticks=plan.raw_off_ticks.tolist(),review_center_reused=plan.review_center_reused)
        Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return validate_review(plan,output)
    process_screen(request,output,checker=ROOT/'screen_pitch_slow_yaw_review_20261029.py',
        failure_details=lambda p:'current empty preparation or continuous-cube evidence rejected',**kwargs)
    return validate_review(plan,output)
