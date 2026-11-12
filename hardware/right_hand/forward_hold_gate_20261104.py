"""Separate observed-reference reproduction from physical90/hold success."""
import json
from pathlib import Path
import numpy as np
import four_reply_yaw_gate_20261029 as transport
from calibrated_quarter_gate_20261028 import bounded
from single_forward_gate_20261030 import validate_model_cases
from forward_hold_plan_20261104 import ROOT,OUT,REQUEST,PROOF,LOADED,ForwardHoldPlan,hashes,sha,validate_candidate,HOLD_SAMPLES
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen
from verified_off_restore_context_20261105 import POLICY as OFF_RESTORE_POLICY
def models():
    z=validate_candidate();path=OUT/'loaded_hand_dynamics/review.json';h=json.loads(path.read_bytes());expected={(f,g) for f in (-15,-10,-5) for g in (.5,1,1.5)}
    if h.get('source_sha256')!=sha(LOADED) or len(h['cases'])!=9 or {(c['book_front_mm'],c['gain_scale']) for c in h['cases']}!=expected or not h.get('cube_disabled') or not h.get('finite_actuators') or h.get('hand_qpos_prescribed') is not False:raise RuntimeError('Hand evidence missing/unbound')
    for c in h['cases']:
        if not c['passed'] or c['stop'] is not None or c['samples_completed']!=len(z['goals'])+30 or not bounded(c,dict(max_self_mm=1e-6,raw_max_self_mm=1e-6,max_support_mm=.15,max_tracking_rad=.18,unapproved_range_excess_rad=1e-6)):raise RuntimeError('Hand safety failed')
    cases=[];sources=[]
    for f in (-15,-10,-5):
        path=OUT/f'material_front_{f}/review.json';r=json.loads(path.read_bytes())
        if r.get('candidate_sha256')!=sha(LOADED) or r.get('model_source_sha256')!=sha(ROOT/'screen_yaw_roundtrip_model_20261030.py') or r.get('simulation_timestep_s')!=.001 or not r.get('initial_placement_also_checked') or r.get('cube_seed_json') is not None or r.get('goals')!=len(z['goals']):raise RuntimeError('Cube evidence source/scope mismatch')
        cases.extend(r['cases']);sources.append(dict(path=path.relative_to(ROOT).as_posix(),sha256=sha(path)))
    validate_model_cases(cases)
    if any(c['last_sample']!=len(z['goals']) for c in cases):raise RuntimeError('Incomplete forward or hold')
    return dict(hand_cases=9,cube_cases=27,all_safety_passed=True,cube_sources=sources,hand_sha256=sha(OUT/'loaded_hand_dynamics/review.json'),model_angle_range_deg=[min(c['final_yaw_from_placement_deg'] for c in cases),max(c['final_yaw_from_placement_deg'] for c in cases)],physical90_verified=False,physical_body_hold_verified=False)
def make_review(p,empty,before):
    z=validate_candidate();loaded=models();r=transport.make_review(p,empty,transport.hashes());after=hashes()
    if before!=after:raise RuntimeError('Sources changed during review')
    for k in ('loaded_model_cases','loaded_model_all_passed','loaded_model_nominal_angle_deg','loaded_geometry_cases','completed_physical_reference'):r['historical_transport_'+k]=r.pop(k)
    if p.SAMPLE_PERIOD!=.075 or p.ON_TIMEOUT!=300 or p.EMPTY_RELEASE_ONLY or p.CONTACT_RELEASE_BEFORE_PHASE is not None:raise RuntimeError('Existing timing/health scope changed')
    r.update(status='supervised_same_forward_fixed_hold_review',code_hashes=after,forward_hold_models=loaded,loaded_goals=len(z['goals']),new_joint_goal_vectors=0,fixed_hold_samples=HOLD_SAMPLES,fixed_hold_seconds=HOLD_SAMPLES*.075,automatic_open=False,automatic_inverse=False,loaded_motion_authorized=True,scope='Same physically observed forward; fixed10s; user supports cube/SPACE OFF. Exact90 and grip strength remain unverified.',physical90_verified=False,physical_body_hold_verified=False,no_live_policy=True,assistant_motor_writes=0)
    r['post_off_restore_policy']=dict(OFF_RESTORE_POLICY)
    return r
def validate_review(p,path):
    r=json.loads(Path(path).read_bytes());expected=make_review(p,r['empty_recovery'],hashes())
    if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('Proof/source/scope mismatch')
    return r
def run_screen(request,output,**kwargs):
    p=ForwardHoldPlan(request);ref=ForwardHoldPlan(REQUEST);r=validate_review(ref,PROOF)
    if np.array_equal(p.source_position(),ref.source_position()):
        r=dict(r);r.update(source_sha256=p.SOURCE_SHA,raw_off_ticks=p.raw_off_ticks.tolist(),review_center_reused=p.review_center_reused);Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return validate_review(p,output)
    process_screen(request,output,checker=ROOT/'screen_forward_hold_current_20261104.py',failure_details=lambda p:'current EMPTY recovery rejected',**kwargs)
    return validate_review(p,output)
