"""Check immutable model evidence before operator-triggered setting writes."""
import json,math
from pathlib import Path
import numpy as np
from horizontal_quarter_plan_20261027 import (HorizontalQuarterPlan,ROOT,REQUEST,PROOF,LOADED_PATH,LOADED_REVIEW,STATIC_REVIEW,hashes,prefix_digest)
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from book_empty_preparation_plan import FRONTS,offsets
from thumb_root_off_screen_process_20261026 import run_screen as process_screen
from horizontal_preload_confirmation_20261027 import POLICY as PRELOAD_POLICY
from horizontal_goal_confirmation_20261027 import POLICY as GOAL_POLICY
from horizontal_empty_recovery_20261027 import POLICY as ROOT_POLICY
from horizontal_empty_recovery_20261027 import RING_CLEAR
from rewired_initial_pair_scope_20261027 import POLICY as INITIAL_PAIR_POLICY
from horizontal_thumb_book_numeric_20261027 import review as thumb_book_review
from horizontal_placement_evidence_20261027 import review as placement_review
from horizontal_receive_resync_20261027 import POLICY as RECEIVE_POLICY
from horizontal_sync_receive_window_20261028 import POLICY as WINDOW_POLICY
from horizontal_read_recovery_20261028 import POLICY as EPISODE_POLICY

def number(value):return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value) and value>=0
def bounded(row,limits):
    return all(number(row.get(key)) and row[key]<=maximum for key,maximum in limits.items())

def separation_review(plan,empty):
    rows=plan.plan(plan.source_position());enabled=any(p==RING_CLEAR for p,q in rows)
    statics=empty.get('static_cases',[]);dynamics=empty.get('dynamic_cases',[])
    if not enabled:
        if any(row.get('initial_pair_scope_enabled') is not False or row.get('initial_pair_scope_valid') is not True or not bounded(row,{'initial_pair_mm':0.,'initial_pair_increase_mm':1e-6,'last_ring_pair_mm':1e-6}) for row in statics+dynamics):raise RuntimeError('Unexpected initial self-contact exception')
        if any(not bounded(row,{'initial_pair_step_increase_mm':1e-6}) for row in statics):raise RuntimeError('Unexpected initial pair step increase without a ring escape')
        return dict(enabled=False,policy=dict(INITIAL_PAIR_POLICY))
    for row in statics+dynamics:
        if row.get('initial_pair_scope_enabled') is not True or row.get('initial_pair_scope_valid') is not True or not bounded(row,{'initial_pair_mm':2.,'maximum_raw_self_mm':2.,'initial_pair_increase_mm':1e-6,'last_ring_pair_mm':1e-6}):raise RuntimeError('Unbounded/deeper initial pair or uncleared ring separation')
    for row in statics:
        if not bounded(row,{'initial_pair_step_increase_mm':1e-6}) or not isinstance(row.get('ring_clear_sample'),int) or not 0<row['ring_clear_sample']<=48 or not isinstance(row.get('ring_clear_goal_ticks'),int) or not plan.source_position()[13]-4<=row['ring_clear_goal_ticks']<=plan.source_position()[13]+100:raise RuntimeError('Missing monotonic bounded initial pair separation')
    required=max(row['ring_clear_goal_ticks'] for row in statics)+4
    if any(not number(row.get('ring_rotation_after_clearance_ticks')) or row['ring_rotation_after_clearance_ticks']<required for row in dynamics):raise RuntimeError('Actual model ring angle did not clear before roots')
    return dict(enabled=True,policy=dict(INITIAL_PAIR_POLICY),required_actual_ring_rotation_ticks=int(required),ring_clearance_goal_ticks=int([q for p,q in rows if p==RING_CLEAR][-1][13]),maximum_initial_pair_mm=max(row['initial_pair_mm'] for row in statics),classification='Only the original pair during initial_hold/ENTRY/RING; every later and other self contact keeps the original zero criterion')

def validate_loaded():
    import hashlib
    loaded=json.loads(LOADED_REVIEW.read_bytes());static=json.loads(STATIC_REVIEW.read_bytes());rows=loaded.get('cases',[]);geometry=static.get('cases',[])
    if loaded.get('all_passed') is not True or loaded.get('finite_stiffness_contact_forces_preserved') is not True or loaded.get('physical_yaw_verified') is not False or loaded.get('hardware_execution_allowed') is not False or len(rows)!=27:raise RuntimeError('Large-turn loaded evidence is incomplete')
    if loaded.get('loaded_path_sha256')!=hashlib.sha256(LOADED_PATH.read_bytes()).hexdigest():raise RuntimeError('Loaded targets differ from their physics review')
    expected={(f,g,mu) for f in FRONTS for g in (.5,1.,1.5) for mu in (.2,.3,.5)}
    if {(v.get('book_front_mm'),v.get('gain_scale'),v.get('friction')) for v in rows}!=expected:raise RuntimeError('Missing or duplicated loaded model conditions')
    for row in rows:
        if row.get('passed') is not True or row.get('stop') is not None or row.get('external_cube_contact') is not False or row.get('final_cube_supported_by_hand') is not True or not bounded(row,{'max_self_mm':1e-6,'max_support_mm':.15,'max_cube_mm':1,'max_tracking_rad':.18,'unapproved_range_excess_rad':1e-6,'center_translation_mm':80,'longest_no_hand_contact_seconds':.25}):raise RuntimeError('Loaded turn/release rejected')
    if static.get('all_passed') is not True or static.get('all_offsets_requested') is not True or len(geometry)!=201 or static.get('source')!=LOADED_PATH.relative_to(ROOT).as_posix():raise RuntimeError('Missing loaded commanded-geometry review')
    if {(v.get('book_front_mm'),tuple(v.get('offset_ticks',[]))) for v in geometry}!={(f,tuple(o)) for f in FRONTS for o in offsets()}:raise RuntimeError('Missing loaded geometry offsets')
    if any(v.get('passed') is not True or not bounded(v,{'max_self_mm':1e-6,'max_support_mm':.15}) for v in geometry):raise RuntimeError('Loaded commanded geometry rejected')
    nominal=next(v for v in rows if (v['book_front_mm'],v['gain_scale'],v['friction'])==(-10.,1.,.3))
    if not isinstance(nominal.get('yaw_before_release_deg'),(int,float)) or not math.isfinite(nominal['yaw_before_release_deg']) or not 85<=nominal['yaw_before_release_deg']<=95 or not bounded(nominal,{'yaw_tilt_change_deg':15}):raise RuntimeError('Nominal model does not approach the quarter-turn target')
    return loaded,static

def validate_empty(plan,empty):
    if empty.get('all_passed') is not True:
        static=empty.get('static_cases',[]);dynamic=empty.get('dynamic_cases',[])
        raise RuntimeError(f"EMPTY recovery model rejected: {sum(v.get('passed') is not True for v in static)}/{len(static)} static and {sum(v.get('passed') is not True for v in dynamic)}/{len(dynamic)} dynamic cases failed; see empty_recovery_screen/review.json")
    if empty.get('start_ticks')!=plan.source_position().tolist():raise RuntimeError('Fresh recovery start differs from its screened encoder readings')
    if empty.get('prefix_sha256')!=prefix_digest(plan):raise RuntimeError('Fresh recovery command path differs from its screen')
    statics=empty.get('static_cases',[]);dynamics=empty.get('dynamic_cases',[])
    if len(statics)!=201 or len(dynamics)!=9:raise RuntimeError('Missing fresh recovery cases')
    if {(v.get('book_front_mm'),tuple(v.get('offset_ticks',[]))) for v in statics}!={(f,tuple(o)) for f in FRONTS for o in offsets()} or {(v.get('book_front_mm'),v.get('gain_scale')) for v in dynamics}!={(f,g) for f in FRONTS for g in (.5,1.,1.5)}:raise RuntimeError('Missing or duplicated fresh recovery conditions')
    for row in statics:
        if row.get('passed') is not True or not bounded(row,{'max_self_mm':1e-6,'max_support_after_release_mm':.15,'initial_support_increase_mm':1e-6,'step_support_increase_mm':1e-6}) or not isinstance(row.get('root_clear_sample'),int) or not 0<row['root_clear_sample']<=70 or not isinstance(row.get('root_clear_goal_ticks'),list) or len(row['root_clear_goal_ticks'])!=3:raise RuntimeError('Fresh static recovery rejected')
    for row in dynamics:
        if row.get('passed') is not True or not bounded(row,{'max_self_mm':1e-6,'max_support_mm':.15,'max_tracking_rad':.16,'post_release_tracking_rad':.12,'unapproved_range_excess_rad':1e-6,'initial_support_increase_mm':1e-6,'root_clear_seconds':3.575}):raise RuntimeError('Fresh dynamic recovery rejected')
    separation_review(plan,empty)
    thumb_book_review(plan,empty)
    return empty

def make_review(plan,empty,before):
    loaded,_=validate_loaded();validate_empty(plan,empty);after=hashes()
    if before!=after:raise RuntimeError('Control/model sources changed during review')
    rows=plan.plan(plan.source_position());case=next(v for v in loaded['cases'] if (v['book_front_mm'],v['gain_scale'],v['friction'])==(-10.,1.,.3))
    return dict(status='horizontal_quarter_operator_review',accepted_for_operator_trial=True,plan_version=plan.PLAN_VERSION,source_sha256=plan.SOURCE_SHA,start_ticks=plan.source_position().tolist(),plan_sha256=plan.plan_digest(plan.source_position()),last_goal_ticks=rows[-1][1].tolist(),mapping=list(SIM_TO_MOTOR_IDS),code_hashes=after,code_unchanged=True,empty_recovery=empty,initial_self_separation=separation_review(plan,empty),held_thumb_book_numerical_policy=thumb_book_review(plan,empty),off_preload_confirmation_policy=dict(PRELOAD_POLICY),goal_readback_confirmation_policy=dict(GOAL_POLICY),empty_root_motion_policy=dict(ROOT_POLICY),stable_placement_review=placement_review(),read_resynchronization_policy=dict(RECEIVE_POLICY),sync_receive_window_policy=dict(WINDOW_POLICY),read_recovery_episode_policy=dict(EPISODE_POLICY),nominal_model_yaw_deg=case['yaw_before_release_deg'],model_yaw_range_deg=[min(r['yaw_before_release_deg'] for r in loaded['cases']),max(r['yaw_before_release_deg'] for r in loaded['cases'])],model_responses_vary=True,robust_90_degree_rotation_verified=False,physical_horizontal_90_verified=False,assistant_motor_writes=0,goal_step_ticks=2,sample_period_s=.075,active_motion_limit_s=300,current_ma=450,hardware_tracking_stop_rad=.18,hardware_health_thresholds_unchanged=True,passive_id4_reentry_only=True)

def validate_review(plan,path):
    report=json.loads(Path(path).read_bytes());rows=plan.plan(plan.source_position())
    expected=dict(accepted_for_operator_trial=True,plan_version=plan.PLAN_VERSION,source_sha256=plan.SOURCE_SHA,start_ticks=plan.source_position().tolist(),plan_sha256=plan.plan_digest(plan.source_position()),last_goal_ticks=rows[-1][1].tolist(),mapping=list(SIM_TO_MOTOR_IDS),code_hashes=hashes(),code_unchanged=True,initial_self_separation=separation_review(plan,report['empty_recovery']),held_thumb_book_numerical_policy=thumb_book_review(plan,report['empty_recovery']),off_preload_confirmation_policy=dict(PRELOAD_POLICY),goal_readback_confirmation_policy=dict(GOAL_POLICY),empty_root_motion_policy=dict(ROOT_POLICY),stable_placement_review=placement_review(),read_resynchronization_policy=dict(RECEIVE_POLICY),sync_receive_window_policy=dict(WINDOW_POLICY),read_recovery_episode_policy=dict(EPISODE_POLICY),goal_step_ticks=2,sample_period_s=.075,active_motion_limit_s=300,current_ma=450,hardware_tracking_stop_rad=.18,hardware_health_thresholds_unchanged=True,passive_id4_reentry_only=True,physical_horizontal_90_verified=False,assistant_motor_writes=0)
    if any(report.get(k)!=v for k,v in expected.items()):raise RuntimeError('Large-turn source/queue/bounds review mismatch; no writes')
    validate_loaded();validate_empty(plan,report['empty_recovery']);return report

def run_screen(request,output,**kwargs):
    plan=HorizontalQuarterPlan(request);reference=HorizontalQuarterPlan(REQUEST);validate_review(reference,PROOF)
    if np.array_equal(plan.source_position(),reference.source_position()):
        report=dict(validate_review(reference,PROOF));report['source_sha256']=plan.SOURCE_SHA;report['exact_start_cached_review_used']=True;Path(output).write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');return validate_review(plan,output)
    process_screen(request,output,checker=ROOT/'screen_horizontal_quarter_20261027.py',failure_details=lambda p:'current EMPTY recovery or immutable large-turn review rejected',**kwargs)
    return validate_review(plan,output)
