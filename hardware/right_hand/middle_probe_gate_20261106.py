"""Require all original safety cases before this supervised clearance probe."""
import json
from pathlib import Path
import numpy as np
import natural_cradle_grasp_gate_20261030 as original
from natural_cradle_grasp_plan_20261030 import hashes as original_hashes,SEED
from calibrated_quarter_gate_20261028 import bounded
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen
from verified_off_restore_context_20261105 import POLICY as RESTORE_POLICY
from middle_probe_plan_20261106 import R,O,REQUEST,PROOF,CANDIDATE,ALIAS,MiddleProbePlan,sha,hashes,validate_candidate
def models():
    z=validate_candidate();hpath=O/'loaded_hand_dynamics/review.json';h=json.loads(hpath.read_bytes());expected={(f,g) for f in (-15,-10,-5) for g in (.5,1.,1.5)}
    if h['source_sha256']!=sha(CANDIDATE) or len(h['cases'])!=9 or {(c['book_front_mm'],c['gain_scale']) for c in h['cases']}!=expected or not h.get('cube_disabled') or not h.get('finite_actuators') or h.get('hand_qpos_prescribed') is not False:raise RuntimeError('Middle probe hand proof missing/unbound')
    for c in h['cases']:
        if not c['passed'] or c['stop'] is not None or c['samples_completed']!=len(z['goals'])+30 or not bounded(c,dict(max_self_mm=1e-6,raw_max_self_mm=1e-6,max_support_mm=.15,max_tracking_rad=.18,unapproved_range_excess_rad=1e-6)):raise RuntimeError('Middle probe hand safety failed')
    cases=[];sources=[]
    for f in (-15,-10,-5):
        path=O/f'free_front_{f}/review.json';r=json.loads(path.read_bytes())
        if r['candidate_sha256']!=sha(ALIAS) or r['model_source_sha256']!=sha(R/'screen_natural_grasp_hold_20261030.py') or r['cube_seed_sha256']!=sha(SEED) or r['simulation_timestep_s']!=.001 or r['goals']!=len(z['goals']):raise RuntimeError('Middle probe cube proof source mismatch')
        cases.extend(r['cases']);sources.append(dict(path=path.relative_to(R).as_posix(),sha256=sha(path)))
    if len(cases)!=27 or {(c['book_front_mm'],c['gain_scale'],c['friction']) for c in cases}!={(f,g,mu) for f in (-15,-10,-5) for g in (.5,1.,1.5) for mu in (.2,.3,.5)}:raise RuntimeError('Middle probe full27 coverage missing')
    for c in cases:
        if not c['passed'] or c['stop'] is not None or c['last_sample']!=len(z['goals']) or not c['initial_placement_passed'] or c['external_cube_contact'] or not c['final_cube_supported_by_hand'] or not c['vertical_seating_line_upper_verified_free'] or not bounded(c,dict(max_self_mm=1e-6,max_support_mm=.15,max_cube_mm=1,max_tracking_rad=.18,unapproved_range_excess_rad=1e-6,longest_no_hand_contact_seconds=.25,center_translation_mm=80)):raise RuntimeError('Middle probe cube safety failed')
        hold=c['grasp_hold_observation']
        if c['gain_scale']==1. and c['friction']==.3 and (not hold['passed'] or hold['evaluated_steps']<4000 or hold['max_yaw_drift_deg']>=3 or hold['max_center_shift_mm']>=2 or min(hold['opposing_contact_fraction'].values())<=.8):raise RuntimeError('Initial nominal grasp not retained')
    failures=[dict(front=c['book_front_mm'],gain=c['gain_scale'],friction=c['friction'],observation=c['grasp_hold_observation']) for c in cases if not c['grasp_hold_observation']['passed']]
    return dict(hand_safety_cases=9,cube_safety_cases=27,sources=sources,hand_review_sha256=sha(hpath),initial_nominal_grasp_task_passed=True,initial_grasp_task_passed_cases=27-len(failures),initial_nonnominal_grasp_task_failures=failures,new_common_grip_verified=False,probe_contact_strength_not_certified=True,upper_layer_access_verified=False,physical_success=False,scope='middle-only diagnostic; unchanged thumb grip; original source grasp/release; all27 cube safety cases mandatory')
def make_review(plan,empty,before):
    model=models();r=original.make_review(plan,empty,original_hashes());after=hashes()
    if before!=after:raise RuntimeError('Probe sources changed during review')
    r['historical_source_grasp_goal_count']=r.pop('grasp_only_loaded_goals');r.update(status='supervised_middle_only_clearance_probe_review',code_hashes=after,middle_probe_models=model,scope='known grasp -> middle-only bend/5s observation -> return to known grasp -> original OPEN -> REMOVE/OFF',loaded_goals=len(validate_candidate()['goals']),probe_motor_ids=[9,10,11,12],thumb_fixed_during_probe=True,manual_joint_teaching=False,rotation_targets=0,physical_rotation_verified=False,physical_grasp_verified=False,new_common_grip_verified=False,post_off_restore_policy=dict(RESTORE_POLICY),loaded_motion_authorized=True)
    return r
def validate_review(plan,path):
    r=json.loads(Path(path).read_bytes());expected=make_review(plan,r['empty_recovery'],hashes())
    if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('Middle probe proof/source/scope mismatch')
    return r
def run_screen(request,output,**kwargs):
    plan=MiddleProbePlan(request);reference=MiddleProbePlan(REQUEST);r=validate_review(reference,PROOF)
    if np.array_equal(plan.source_position(),reference.source_position()):
        value=make_review(plan,r['empty_recovery'],hashes());Path(output).write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8');return validate_review(plan,output)
    process_screen(request,output,checker=R/'screen_middle_probe_current_20261106.py',failure_details=lambda p:'fresh middle-probe EMPTY recovery rejected',**kwargs);return validate_review(plan,output)
