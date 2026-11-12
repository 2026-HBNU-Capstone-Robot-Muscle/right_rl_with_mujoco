"""Verify the complete no-yaw observation queue and its source-bound evidence."""
import hashlib,json
from pathlib import Path
import numpy as np
from pitch_handoff_plan_20261028 import (PitchHandoffPlan,ROOT,OUT,REQUEST,PROOF,
    PITCH,TRANSFER,LOADED_PATH,SOURCE_REVIEW,RING_SIDE_DELTA,INDEX_SIDE_DELTA,TRANSFER_PATH,TRANSFER_SEED,
    loaded_rows,pitch_start,hashes)
from calibrated_quarter_gate_20261028 import (validate_empty,separation_review,bounded,
    PRELOAD_POLICY,GOAL_POLICY,ROOT_POLICY,RECEIVE_POLICY,WINDOW_POLICY,EPISODE_POLICY,CLOCK_POLICY)
from calibrated_empty_ring_margin_20261028 import review as margin_review
from horizontal_thumb_book_numeric_20261027 import review as thumb_book_review
from horizontal_serial_link_20261028 import metadata as link_review
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from book_empty_preparation_plan import FRONTS,offsets
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen
from slow_thumb_seam_resolution_20261026 import metadata as seam_metadata

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def read(path):return json.loads(path.read_bytes())

def validate_loaded():
    source=read(SOURCE_REVIEW)
    if source.get('physical_successes')!=2 or source.get('asset_sha256')!=sha(PITCH):raise RuntimeError('Historical pitch asset changed')
    for row in source['sources']:
        path=ROOT/row['source']
        if sha(path/'result.json')!=row['raw_result_sha256'] or sha(path/'visual_review/physical_flip_review.json')!=row['visual_evidence_sha256']:raise RuntimeError('Historical physical evidence changed')
    z=np.load(LOADED_PATH);pitch=np.load(PITCH);rows=loaded_rows();n=len(pitch['goals'])
    np.testing.assert_array_equal(z['start'],pitch_start())
    np.testing.assert_array_equal(z['goals'],[q for p,q in rows])
    np.testing.assert_array_equal(z['phases'],[p for p,q in rows])
    diff=z['goals'][:n]-pitch['goals'];expected=np.zeros_like(diff);expected[:,5]=-8;expected[:,13]=8
    if (INDEX_SIDE_DELTA,RING_SIDE_DELTA)!=(-8,8) or not np.array_equal(diff,expected):raise RuntimeError('Pitch differs beyond the two scoped axial corrections')
    transfer=np.load(TRANSFER_PATH)
    np.testing.assert_array_equal(transfer['start'],z['goals'][n-1])
    np.testing.assert_array_equal(transfer['goals'],z['goals'][n:n+600])
    if len(rows)!=1908:raise RuntimeError('Unexpected loaded queue; subsequent yaw is forbidden')
    geometry=read(OUT/'loaded_geometry/review.json');hand=read(OUT/'loaded_hand_dynamics/review.json')
    retention=read(OUT/'transfer_retention/review.json')
    if geometry.get('source')!=LOADED_PATH.relative_to(ROOT).as_posix() or not geometry.get('all_offsets_requested') or geometry.get('all_passed') is not True:raise RuntimeError('Loaded commanded geometry rejected')
    if {(r['book_front_mm'],tuple(r['offset_ticks'])) for r in geometry['cases']}!={(f,tuple(o)) for f in FRONTS for o in offsets()}:raise RuntimeError('Incomplete loaded geometry conditions')
    if any(r.get('passed') is not True or not bounded(r,{'max_self_mm':1e-6,'max_support_mm':.15}) for r in geometry['cases']):raise RuntimeError('Loaded geometry bounds violated')
    if hand.get('source_sha256')!=sha(LOADED_PATH) or hand.get('all_passed') is not True or hand.get('cube_disabled') is not True:raise RuntimeError('Loaded joint-route dynamics rejected')
    if hand.get('existing_palm7_seam_policy')!=seam_metadata() or hand.get('gravity_up_world')!=read(TRANSFER_SEED)['up_world']:raise RuntimeError('Existing palm seam or gravity reference changed')
    if len(hand['cases'])!=9 or {(r['book_front_mm'],r['gain_scale']) for r in hand['cases']}!={(f,g) for f in FRONTS for g in (.5,1.,1.5)}:raise RuntimeError('Incomplete hand dynamics')
    if any(r.get('passed') is not True or r.get('stop') is not None or not bounded(r,{'max_self_mm':1e-6,'max_support_mm':.15,'max_tracking_rad':.18,'unapproved_range_excess_rad':1e-6}) for r in hand['cases']):raise RuntimeError('Joint-route dynamics bounds violated')
    if any(not bounded(r,{'existing_palm7_seam_discarded_max_mm':.01}) for r in hand['cases']):raise RuntimeError('Existing seam classification exceeded')
    if retention.get('all_retention_cases_passed') is not True or retention.get('subsequent_yaw_not_allowed') is not True:raise RuntimeError('Transfer retention not accepted for observation')
    if retention.get('transfer_path_sha256')!=sha(TRANSFER_PATH) or retention.get('initial_cube_seed_sha256')!=sha(TRANSFER_SEED):raise RuntimeError('Transfer retention source mismatch')
    cases=retention['cases']
    if len(cases)!=27 or {(v['book_front_mm'],v['scale'],v['friction']) for v in cases}!={(f,g,mu) for f in FRONTS for g in (.5,1.,1.5) for mu in (.2,.3,.5)}:raise RuntimeError('Incomplete transfer conditions')
    for row in cases:
        if row.get('accepted_for_transfer_observation') is not True or row.get('stop') is not None or row.get('external_cube_contact') is not False or row['acceptance_checks'].get('final_hand_contact') is not True or not bounded(row,{'max_self_mm':1e-6,'max_support_mm':.15,'max_cube_mm':1,'max_tracking_rad':.18,'native_range_excess_rad':1e-6,'max_no_hand_contact_seconds':.25,'red_up_deg':15}):raise RuntimeError('Transfer bounds violated')
    return dict(historical_pitch_successes=2,recorded_pitch_goals=n,
        scoped_pitch_correction=dict(motor_ids=[6,14],delta_ticks=[-8,8],other14_joint_goals_unchanged=True),
        commanded_geometry_cases=201,hand_dynamics_cases=9,conditional_transfer_cases=27,
        initial_cube_pose_not_registered=True,physical_pitch_with_correction_not_yet_verified=True,
        physical_transfer_not_yet_verified=True,ready_for_next_yaw_grasp=False,subsequent_yaw_forbidden=True,
        loaded_geometry_sha256=sha(OUT/'loaded_geometry/review.json'),
        hand_dynamics_sha256=sha(OUT/'loaded_hand_dynamics/review.json'),
        transfer_retention_sha256=sha(OUT/'transfer_retention/review.json'))

def make_review(plan,empty,before):
    validate_empty(plan,empty);loaded=validate_loaded();after=hashes()
    if before!=after:raise RuntimeError('Code or fixed evidence changed during review')
    rows=plan.plan(plan.source_position())
    return dict(status='pitch_then_transfer_supervised_observation_review',accepted_for_operator_trial=True,
        plan_version=plan.PLAN_VERSION,source_sha256=plan.SOURCE_SHA,start_ticks=plan.source_position().tolist(),
        raw_off_ticks=plan.raw_off_ticks.tolist(),review_center_reused=plan.review_center_reused,
        reviewed_start_radius_ticks=4,review_center_source_sha256=sha(REQUEST),
        plan_sha256=plan.plan_digest(plan.source_position()),last_goal_ticks=rows[-1][1].tolist(),mapping=list(SIM_TO_MOTOR_IDS),
        code_hashes=after,code_unchanged=True,empty_recovery=empty,loaded_evidence=loaded,
        initial_self_separation=separation_review(plan,empty),empty_ring_pip_margin=margin_review(plan),
        held_thumb_book_numerical_policy=thumb_book_review(plan,empty),
        off_preload_confirmation_policy=dict(PRELOAD_POLICY),goal_readback_confirmation_policy=dict(GOAL_POLICY),
        empty_root_motion_policy=dict(ROOT_POLICY),read_resynchronization_policy=dict(RECEIVE_POLICY),
        sync_receive_window_policy=dict(WINDOW_POLICY),read_recovery_episode_policy=dict(EPISODE_POLICY),
        precise_clock_policy=dict(CLOCK_POLICY),verified_serial_link_policy=link_review(),
        active_motion_limit_s=300,goal_step_ticks=2,sample_period_s=.075,current_ma=450,
        maximum_aggregate_thumb_displacement_ticks=1568,native_joint_ranges_unchanged=True,
        other_hardware_health_stops_unchanged=True,no_live_policy=True,next_yaw_queued=False,
        physical_new_motion_verified=False,assistant_motor_writes=0)

def validate_review(plan,path):
    report=read(Path(path));expected=make_review(plan,report['empty_recovery'],hashes())
    if any(report.get(k)!=v for k,v in expected.items()):raise RuntimeError('Pitch/transfer source, bounds or review mismatch; no writes')
    return report

def run_screen(request,output,**kwargs):
    plan=PitchHandoffPlan(request);reference=PitchHandoffPlan(REQUEST)
    reviewed=validate_review(reference,PROOF)
    if np.array_equal(plan.source_position(),reference.source_position()):
        reviewed=dict(reviewed);reviewed.update(source_sha256=plan.SOURCE_SHA,
            raw_off_ticks=plan.raw_off_ticks.tolist(),review_center_reused=plan.review_center_reused)
        print('Using the existing reviewed4-tick start neighborhood; fresh OFF health remains checked.',flush=True)
        Path(output).write_text(json.dumps(reviewed,indent=2)+'\n',encoding='utf-8')
        return validate_review(plan,output)
    process_screen(request,output,checker=ROOT/'screen_pitch_handoff_review_20261028.py',
                   failure_details=lambda p:'current EMPTY pitch setup or fixed transfer evidence rejected',**kwargs)
    return validate_review(plan,output)
