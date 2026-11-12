"""Actual forward sequence, unchanged static poses, finite EMPTY reverse dynamics."""
import hashlib,json
from pathlib import Path
import numpy as np
import four_reply_yaw_gate_20261029 as base
from reverse_yaw_empty_plan_20261029 import ReverseYawEmptyPlan,ROOT,OUT,REQUEST,PROOF,HAND_PATH,LOADED_PATH,FORWARD_RESULT,hashes,reverse_rows
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen
def reverse_source():
    old=np.load(LOADED_PATH);z=np.load(HAND_PATH);count=len(old['goals'])
    if not np.array_equal(z['goals'][:count],old['goals'][::-1]):raise RuntimeError('Reverse cycle is not the exact original in reverse order')
    original={tuple(q) for q in np.vstack([old['start'],old['goals']])}
    if any(tuple(q) not in original for q in z['goals']):raise RuntimeError('Reverse introduces a pose outside original201-case geometry')
    if sum(p=='stage_7_horizontal_yaw' for p,q in reverse_rows())!=638:raise RuntimeError('Reverse yaw incomplete')
    r=json.loads(FORWARD_RESULT.read_bytes())
    if r['status']!='adaptive_loaded_yaw_observation_complete' or not r['all_torque_off_verified'] or not r['settings_restored']:raise RuntimeError('Completed forward source unavailable')
    from calibrated_quarter_plan_20261028 import loaded_rows
    written=[e['requested'] for e in r['write_events'] if e['address']==116]
    if written[-count:]!=[q.tolist() for p,q in loaded_rows()]:raise RuntimeError('Forward source was not actually written in the original order')
    return dict(path=FORWARD_RESULT.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(FORWARD_RESULT.read_bytes()).hexdigest(),
        original_cycle_goals=count,exact_reverse_order=True,all_commanded_poses_already_in_original_geometry=True,
        original_angles_current_PID_and75ms_pacing=True,physical_inverse_cube_rotation_not_yet_reviewed=True)
def hand_review():
    path=OUT/'loaded_hand_dynamics/review.json';r=json.loads(path.read_bytes())
    if r.get('source_sha256')!=hashlib.sha256(HAND_PATH.read_bytes()).hexdigest() or not r.get('all_passed') or not r.get('cube_disabled') or not r.get('finite_actuators'):raise RuntimeError('EMPTY reverse dynamics rejected')
    if len(r['cases'])!=9 or {(c['book_front_mm'],c['gain_scale']) for c in r['cases']}!={(f,g) for f in (-15.,-10.,-5.) for g in (.5,1.,1.5)}:raise RuntimeError('Missing reverse dynamics conditions')
    for c in r['cases']:
        if not c['passed'] or c['stop'] is not None or c['max_self_mm']>1e-6 or c['max_support_mm']>.15 or c['max_tracking_rad']>.18 or c['unapproved_range_excess_rad']>1e-6:raise RuntimeError('Reverse violates original hand limits')
    return dict(path=path.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),cases=9,cube_disabled=True,finite_actuators=True)
def make_review(p,empty,before):
    r=base.make_review(p,empty,base.hashes());after=hashes()
    if before!=after:raise RuntimeError('Reverse control sources changed during review')
    r['historical_forward_reference']=r.pop('completed_physical_reference')
    r['historical_forward_loaded_model_cases']=r.pop('loaded_model_cases')
    r['historical_forward_loaded_model_all_passed']=r.pop('loaded_model_all_passed')
    r['historical_forward_loaded_model_nominal_angle_deg']=r.pop('loaded_model_nominal_angle_deg')
    r.update(status='EMPTY_reverse_joint_cycle_operator_review',code_hashes=after,reverse_source=reverse_source(),
        reverse_empty_hand_dynamics=hand_review(),cube_stays_OUT=True,no_PLACE_or_REMOVE=True,no_placement_confirmation_fabricated=True,
        no_new_wait_controller=True,loaded_model_evidence_is_historical_only=True,physical_inverse_rotation_verified=False,
        inverse_cube_trial_ready=False,reverse_yaw_goals=638,final_empty_fixed_hold_s=1.5)
    return r
def validate_review(p,path):
    r=json.loads(Path(path).read_bytes());expected=make_review(p,r['empty_recovery'],hashes())
    if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('EMPTY reverse proof/source mismatch')
    return r
def run_screen(request,output,**kwargs):
    p=ReverseYawEmptyPlan(request);ref=ReverseYawEmptyPlan(REQUEST);r=validate_review(ref,PROOF)
    if np.array_equal(p.source_position(),ref.source_position()):
        r=dict(r);r.update(source_sha256=p.SOURCE_SHA,raw_off_ticks=p.raw_off_ticks.tolist(),review_center_reused=p.review_center_reused)
        Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return validate_review(p,output)
    process_screen(request,output,checker=ROOT/'screen_reverse_yaw_empty_20261029.py',failure_details=lambda p:'reverse EMPTY recovery rejected',**kwargs)
    return validate_review(p,output)
