"""Retain real transport/empty safety; explicitly identify the limited model scope."""
import json
from pathlib import Path
import numpy as np
import four_reply_yaw_gate_20261029 as transport
from calibrated_quarter_gate_20261028 import bounded
from verified_off_restore_context_20261105 import POLICY as OFF_POLICY
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen
from nominal_cycle_plan_20261112 import ROOT,OUT,REQUEST,PROOF,SOURCE,SOURCE_REVIEW,NominalCyclePlan,candidate,hashes,sha,scope_approved

def make_review(plan,empty,before):
    r=transport.make_review(plan,empty,transport.hashes())
    nominal=json.loads(SOURCE_REVIEW.read_bytes()); hands=[]
    if (nominal['all1ms_samples']!=5089*75 or len(nominal['holds'])!=2
            or any(h['samples']!=4050 or h['supported_samples']!=4050 or h['upper_certified_samples']!=4050 for h in nominal['holds'])
            or not bounded(nominal['worst'],{'self_mm':1e-6,'support_mm':.15,'cube_mm':1.,'tracking_rad':.18,'range_rad':1e-6})
            or nominal['final180_error_deg']>=5 or nominal['final_tilt_deg']>=5
            or any(any(v!=0 for v in s['bound_state_errors'].values()) for s in nominal['seams'])):
        raise RuntimeError('Archived nominal complete-cycle proof failed')
    for gain in (.5,1.,1.5):
        path=OUT/f'hand_gain{gain:g}.json'; h=json.loads(path.read_bytes())
        if h['source_sha256']!=sha(SOURCE) or h['gain']!=gain or h['every_executed_ms_checked']!=1 or h['native_thresholds_changed']:
            raise RuntimeError('Hand comparison source mismatch')
        hands.append({'gain':gain,'passed':h['passed'],'stop':h['stop'],'worst':h['worst'],'source_sha256':sha(path)})
    if before!=hashes(): raise RuntimeError('Sources changed during review')
    for key in ('loaded_model_cases','loaded_model_all_passed','loaded_model_nominal_angle_deg','loaded_geometry_cases','completed_physical_reference'):
        if key in r: r['historical_transport_'+key]=r.pop(key)
    approved=scope_approved()
    r.update(status='supervised_nominal_complete_cycle_diagnostic_review',code_hashes=hashes(),
             loaded_goals=len(candidate()['goals']),nominal_complete_cycle_source_sha256=sha(SOURCE_REVIEW),
             nominal_model_complete_task_passed=True,hand_only_comparisons=hands,hand_only_all_passed=all(h['passed'] for h in hands),
             perturbation_robustness_passed=False,all_condition_safety_certified=False,physical_two_turn_success=False,
             post_off_restore_policy=dict(OFF_POLICY),active_motion_limit_s=500,
             existing_300s_limit_extension_explicitly_approved=approved,
             accepted_for_operator_trial=approved,loaded_motion_authorized=approved,
             no_live_policy=True,full_task_success_certified=False,automatic_open=False,automatic_inverse=False,
             scope='Archived5089-goal nominal diagnostic; compare actual two turns and two holds; known finite-condition and hand-only failures retained')
    return r

def validate_review(plan,path):
    r=json.loads(Path(path).read_bytes()); expected=make_review(plan,r['empty_recovery'],hashes())
    if any(r.get(k)!=v for k,v in expected.items()): raise RuntimeError('Cycle proof/source/scope mismatch; no writes')
    return r

def run_screen(request,output,**kwargs):
    if not scope_approved(): raise RuntimeError('Explicit500s nominal diagnostic approval required; no writes')
    plan=NominalCyclePlan(request); reference=NominalCyclePlan(REQUEST); old=validate_review(reference,PROOF)
    if np.array_equal(plan.source_position(),reference.source_position()):
        r=make_review(plan,old['empty_recovery'],hashes()); Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8'); return validate_review(plan,output)
    process_screen(request,output,checker=ROOT/'screen_nominal_cycle_current_20261112.py',failure_details=lambda _:'fresh EMPTY recovery rejected',**kwargs)
    return validate_review(plan,output)
