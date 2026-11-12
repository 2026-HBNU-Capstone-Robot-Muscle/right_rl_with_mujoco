"""Bind actual EMPTY motor writes and all27 cube conditions, keeping health stops."""
import json,math
from pathlib import Path
import numpy as np
import four_reply_yaw_gate_20261029 as transport
from supported_inverse_empty_gate_20261029 import new_empty_review
from supported_inverse_trial_plan_20261029 import (SupportedInverseTrialPlan,ROOT,OUT,REQUEST,PROOF,LOADED,HAND_PATH,ACTUAL,ACTUAL_REVIEW,SEED,MODEL_SOURCES,hashes,sha,new_rows)
from calibrated_quarter_gate_20261028 import bounded
from gapped_inverse_plan_20261029 import placement_goal
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen
def actual_empty():
    r=json.loads(ACTUAL.read_bytes());review=json.loads(ACTUAL_REVIEW.read_bytes())
    if review['source_sha256']!=sha(ACTUAL) or review['candidate_sha256']!=sha(LOADED) or review['hand_path_sha256']!=sha(HAND_PATH):raise RuntimeError('Physical EMPTY source/path changed')
    if not review['all_actual_goal_writes_match_ON_rebased_queue'] or not review['all_new_hand_goal_writes_match_bound_path']:raise RuntimeError('Actual EMPTY writes incomplete')
    if r['status']!='supported_inverse_empty_path_complete' or not r['all_torque_off_verified'] or not r['settings_restored'] or r['cube_present'] is not False:raise RuntimeError('Physical EMPTY check incomplete')
    expected=[q.tolist() for _,q in new_rows()];written=[e['requested'] for e in r['write_events'] if e['address']==116]
    if written[-len(expected):]!=expected:raise RuntimeError('New hand trajectory not actually written while EMPTY')
    return dict(path=ACTUAL.relative_to(ROOT).as_posix(),sha256=sha(ACTUAL),review_sha256=sha(ACTUAL_REVIEW),exact_hand_goals=len(expected),cube_was_not_present=True)
def cube_models():
    cases=[];sources=[];z=np.load(LOADED)
    if not np.array_equal(z['start'],placement_goal()):raise RuntimeError('Placement checkpoint differs from supported gap')
    for path in MODEL_SOURCES:
        r=json.loads(path.read_bytes())
        if not r.get('all_passed') or r.get('candidate_sha256')!=sha(LOADED) or r.get('cube_seed_sha256')!=sha(SEED):raise RuntimeError('Cube path/placement model not bound or rejected')
        if r.get('simulation_timestep_s')!=.001 or not r.get('initial_placement_also_checked'):raise RuntimeError('Full1ms physics/placement check missing')
        cases.extend(r['cases']);sources.append(dict(path=path.relative_to(ROOT).as_posix(),sha256=sha(path)))
    if len(cases)!=27 or {(c['book_front_mm'],c['gain_scale'],c['friction']) for c in cases}!={(f,g,mu) for f in (-15.,-10.,-5.) for g in (.5,1.,1.5) for mu in (.2,.3,.5)}:raise RuntimeError('Cube27 conditions incomplete')
    for c in cases:
        if not c['passed'] or c['stop'] is not None or not c['initial_placement_passed'] or c['external_cube_contact'] or not c['final_cube_supported_by_hand']:raise RuntimeError('Cube retention rejected')
        if not bounded(c,{'max_self_mm':1e-6,'max_support_mm':.15,'max_cube_mm':1,'max_tracking_rad':.18,'unapproved_range_excess_rad':1e-6,'center_translation_mm':80,'longest_no_hand_contact_seconds':.25}):raise RuntimeError('Cube physics exceeds original bounds')
        if not math.isfinite(c['placed_to_turn_deg']) or c['placed_to_turn_deg']>-15 or not 0<=c['yaw_tilt_change_deg']<=15:raise RuntimeError('Counter-rotation or upright orientation unsupported')
        probe=c['initial_placement_probe']
        if not bounded(probe,{'max_self_mm':1e-6,'max_support_mm':.15,'max_cube_mm':1,'max_tracking_rad':.18}) or probe['external_cube_contact']:raise RuntimeError('Initial placement violates original bounds')
    nominal=next(c for c in cases if (c['book_front_mm'],c['gain_scale'],c['friction'])==(-10.,1.,.3))
    return dict(sources=sources,cases=27,all_passed=True,nominal_placed_to_hold_deg=nominal['placed_to_turn_deg'],
        range_deg=[min(c['placed_to_turn_deg'] for c in cases),max(c['placed_to_turn_deg'] for c in cases)],
        physical_inverse_90_verified=False,initial_cube_pose_not_registered=True)
def make_review(p,empty,before):
    if p.EMPTY_RELEASE_ONLY or p.CAPTURE_BEFORE_PHASE!='stage_5_loaded_grasp':raise RuntimeError('Genuine PLACE checkpoint changed')
    r=transport.make_review(p,empty,transport.hashes())
    r['historical_forward_reference']=r.pop('completed_physical_reference')
    for key in ('loaded_model_cases','loaded_model_all_passed','loaded_model_nominal_angle_deg','loaded_geometry_cases'):
        r['historical_forward_'+key]=r.pop(key)
    hand=new_empty_review();physical=actual_empty();models=cube_models();after=hashes()
    if before!=after:raise RuntimeError('Supported cube controller/model source changed during review')
    r.update(status='supervised_supported_inverse_cube_observation_review',code_hashes=after,new_hand_models=hand,actual_empty_evidence=physical,
        supported_inverse_cube_models=models,placement_orientation='GREEN up; WHITE wristward',post_PLACE_yaw_goals=int(np.sum(np.load(LOADED)['phases']=='stage_7_horizontal_yaw')),
        no_placement_confirmation_fabricated=True,no_new_wait_controller=True,loaded_model_evidence_is_historical_only=False,
        physical_inverse_90_verified=False,physical_new_cube_trial_completed=False)
    return r
def validate_review(p,path):
    r=json.loads(Path(path).read_bytes());expected=make_review(p,r['empty_recovery'],hashes())
    if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('Supported cube proof/source mismatch')
    return r
def run_screen(request,output,**kwargs):
    p=SupportedInverseTrialPlan(request);ref=SupportedInverseTrialPlan(REQUEST);r=validate_review(ref,PROOF)
    if np.array_equal(p.source_position(),ref.source_position()):
        r=dict(r);r.update(source_sha256=p.SOURCE_SHA,raw_off_ticks=p.raw_off_ticks.tolist(),review_center_reused=p.review_center_reused)
        Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return validate_review(p,output)
    process_screen(request,output,checker=ROOT/'screen_supported_inverse_trial_20261029.py',failure_details=lambda p:'supported cube current EMPTY prefix rejected',**kwargs)
    return validate_review(p,output)
