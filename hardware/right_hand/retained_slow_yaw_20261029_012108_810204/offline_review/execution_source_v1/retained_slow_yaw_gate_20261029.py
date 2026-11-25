"""Fresh empty recovery and unchanged slower-yaw suffix; supervised only."""
import json,hashlib
from pathlib import Path
import numpy as np
from retained_slow_yaw_plan_20261029 import RetainedSlowYawPlan,ROOT,OUT,REQUEST,PROOF,LOADED_PATH,loaded_rows,hashes
from calibrated_quarter_gate_20261028 import (validate_empty,separation_review,bounded,PRELOAD_POLICY,GOAL_POLICY,
    ROOT_POLICY,WINDOW_POLICY,EPISODE_POLICY,CLOCK_POLICY)
from pitch_slow_yaw_gate_20261029 import validate_loaded as combined_evidence
from pitch_slow_yaw_plan_20261029 import loaded_rows as combined_rows
from combined_quiet_reserve_20261029 import POLICY as RECEIVE_POLICY
from calibrated_empty_ring_margin_20261028 import review as margin_review
from horizontal_thumb_book_numeric_20261027 import review as thumb_review
from horizontal_serial_link_20261028 import metadata as link_review
from slow_thumb_seam_resolution_20261026 import metadata as seam_metadata
from book_empty_preparation_plan import FRONTS
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen
def read(p):return json.loads(Path(p).read_bytes())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def validate_loaded():
    reference=combined_evidence();own=loaded_rows();suffix=combined_rows()[-len(own):]
    if [(p,q.tolist()) for p,q in own]!=[(p,q.tolist()) for p,q in suffix]:raise RuntimeError('Slow-yaw suffix differs from reviewed combined path')
    hand=read(OUT/'loaded_hand_dynamics/review.json')
    if hand.get('source_sha256')!=sha(LOADED_PATH) or hand.get('all_passed') is not True or hand.get('cube_disabled') is not True:raise RuntimeError('Isolated hand dynamics missing/rejected')
    if hand.get('existing_palm7_seam_policy')!=seam_metadata():raise RuntimeError('Existing geometry scope changed')
    if len(hand['cases'])!=9 or {(r['book_front_mm'],r['gain_scale']) for r in hand['cases']}!={(f,g) for f in FRONTS for g in (.5,1.,1.5)}:raise RuntimeError('Isolated hand conditions incomplete')
    for r in hand['cases']:
        if r.get('passed') is not True or r.get('stop') is not None or not bounded(r,{'max_self_mm':1e-6,'max_support_mm':.15,
            'max_tracking_rad':.18,'unapproved_range_excess_rad':1e-6,'existing_palm7_seam_discarded_max_mm':.01}):raise RuntimeError('Isolated hand bounds failed')
    return dict(combined_reference=reference,unchanged_slow_yaw_suffix=True,loaded_targets=len(own),
        isolated_hand_review_sha256=sha(OUT/'loaded_hand_dynamics/review.json'),
        initial_orientation='RED up; GREEN toward wrist ID41',manual_placement_is_provisional=True,
        robust_90_degrees_not_certified=True,supervised_experiment_only=True)
def make_review(plan,empty,before):
    validate_empty(plan,empty);loaded=validate_loaded();after=hashes()
    if before!=after:raise RuntimeError('Source changed during review')
    rows=plan.plan(plan.source_position())
    return dict(status='supervised_retained_red_slow_yaw_review',accepted_for_operator_trial=True,
        plan_version=plan.PLAN_VERSION,source_sha256=plan.SOURCE_SHA,start_ticks=plan.source_position().tolist(),
        raw_off_ticks=plan.raw_off_ticks.tolist(),review_center_reused=plan.review_center_reused,reviewed_start_radius_ticks=4,
        plan_sha256=plan.plan_digest(plan.source_position()),last_goal_ticks=rows[-1][1].tolist(),mapping=list(SIM_TO_MOTOR_IDS),
        code_hashes=after,code_unchanged=True,empty_recovery=empty,loaded_evidence=loaded,
        initial_self_separation=separation_review(plan,empty),empty_ring_pip_margin=margin_review(plan),held_thumb_book_numerical_policy=thumb_review(plan,empty),
        off_preload_confirmation_policy=dict(PRELOAD_POLICY),goal_readback_confirmation_policy=dict(GOAL_POLICY),
        empty_root_motion_policy=dict(ROOT_POLICY),read_resynchronization_policy=dict(RECEIVE_POLICY),
        sync_receive_window_policy=dict(WINDOW_POLICY),read_recovery_episode_policy=dict(EPISODE_POLICY),precise_clock_policy=dict(CLOCK_POLICY),
        verified_serial_link_policy=link_review(),sample_period_s=.075,distinct_yaw_goal_period_s=.15,goal_step_ticks=2,
        current_ma=450,active_motion_limit_s=300,nominal_motion_seconds=len(rows)*.075,hardware_health_thresholds_unchanged=True,
        no_live_policy=True,physical_retained_red_yaw_verified=False,assistant_motor_writes=0)
def validate_review(plan,path):
    r=read(path);expected=make_review(plan,r['empty_recovery'],hashes())
    if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('Retained-yaw source, queue, bounds or evidence mismatch')
    return r
def run_screen(request,output,**kwargs):
    plan=RetainedSlowYawPlan(request);reference=RetainedSlowYawPlan(REQUEST);r=validate_review(reference,PROOF)
    if np.array_equal(plan.source_position(),reference.source_position()):
        r=dict(r);r.update(source_sha256=plan.SOURCE_SHA,raw_off_ticks=plan.raw_off_ticks.tolist(),review_center_reused=plan.review_center_reused)
        Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return validate_review(plan,output)
    process_screen(request,output,checker=ROOT/'screen_retained_slow_yaw_review_20261029.py',
        failure_details=lambda p:'fresh empty recovery or fixed slower-yaw evidence rejected',**kwargs)
    return validate_review(plan,output)
