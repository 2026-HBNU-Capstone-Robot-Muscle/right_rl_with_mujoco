"""Current recovery and full finite EMPTY diagnostic hand-prefix evidence."""
import json,hashlib
from pathlib import Path
import numpy as np
from yaw_load_empty_plan_20261029 import YawLoadEmptyPlan,ROOT,OUT,REQUEST,PROOF,HAND_PATH,hashes,motion_prefix
import four_reply_yaw_gate_20261029 as base
def hand_review():
    path=OUT/'loaded_hand_dynamics/review.json';r=json.loads(path.read_bytes())
    if r.get('source_sha256')!=hashlib.sha256(HAND_PATH.read_bytes()).hexdigest() or not r.get('all_passed') or not r.get('cube_disabled') or not r.get('finite_actuators'):raise RuntimeError('EMPTY same-path hand dynamics rejected')
    expected={(f,g) for f in (-15.,-10.,-5.) for g in (.5,1.,1.5)}
    if len(r['cases'])!=9 or {(c['book_front_mm'],c['gain_scale']) for c in r['cases']}!=expected:raise RuntimeError('Missing EMPTY hand conditions')
    for c in r['cases']:
        if c['passed'] is not True or c['stop'] is not None or c['max_self_mm']>1e-6 or c['max_support_mm']>.15 or c['max_tracking_rad']>.18 or c['unapproved_range_excess_rad']>1e-6:raise RuntimeError('Original hand bounds failed')
    return dict(path=path.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),cases=9,cube_disabled=True,original_bounds_unchanged=True)
def make_review(p,empty,before):
    r=base.make_review(p,empty,base.hashes());after=hashes()
    if before!=after:raise RuntimeError('Source changed during EMPTY prefix review')
    r.update(status='EMPTY_same_joint_load_comparison_review',code_hashes=after,empty_hand_prefix_dynamics=hand_review(),cube_stays_OUT=True,
        no_PLACE_or_REMOVE=True,no_placement_confirmation_fabricated=True,exact_original_loaded_prefix_used_without_cube=True,
        yaw_targets=320,limited_after_previous_failed_yaw_target267=True,final_fixed_hold_seconds=1.5,
        loaded_model_evidence_is_historical_only=True,physical_cube_force_not_measured=True)
    return r
def validate_review(p,path):
    r=json.loads(Path(path).read_bytes());expected=make_review(p,r['empty_recovery'],hashes())
    if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('EMPTY prefix proof/source/limits mismatch')
    return r
def run_screen(request,output,**kwargs):
    p=YawLoadEmptyPlan(request);reference=YawLoadEmptyPlan(REQUEST);r=validate_review(reference,PROOF)
    if np.array_equal(p.source_position(),reference.source_position()):
        r=dict(r);r.update(source_sha256=p.SOURCE_SHA,raw_off_ticks=p.raw_off_ticks.tolist(),review_center_reused=p.review_center_reused)
        Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return validate_review(p,output)
    base.base.process_screen(request,output,checker=ROOT/'screen_yaw_load_empty_20261029.py',failure_details=lambda p:'EMPTY same-path current recovery rejected',**kwargs)
    return validate_review(p,output)
