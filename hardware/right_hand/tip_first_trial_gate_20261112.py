"""Bind the new transfer and fresh OFF recovery to unchanged transport guards."""
import json
from pathlib import Path
import numpy as np
import four_reply_yaw_gate_20261029 as transport
from calibrated_quarter_gate_20261028 import bounded
from empty_start_thumb_index_scope_20261113 import validate_report as validate_initial_scope
from verified_off_restore_context_20261105 import POLICY as OFF_POLICY
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen
from tip_first_trial_plan_20261112 import ROOT,OUT,REQUEST,PROOF,CANDIDATE,TipFirstTrialPlan,candidate,hashes,sha,scope_approved

def models():
    z=candidate();sources=[];loaded=[];hands=[];checker=ROOT/'screen_tip_first_models_20261112.py'
    for gain in (.5,1.,1.5):
        for hand in (False,True):
            path=OUT/'models'/f'{"hand" if hand else "loaded"}_gain{gain:g}.json';r=json.loads(path.read_bytes())
            if (r['candidate_sha256']!=sha(CANDIDATE) or r['checker_sha256']!=sha(checker) or r['gain']!=gain
                    or r['hand_only']!=hand or r['every_executed_ms_checked']!=1 or r['native_guards_changed']
                    or r['actual_state_resets_after_initial_placement']!=0):raise RuntimeError('Transfer model source/scope mismatch')
            for key in ('model','initial'):
                provenance=r['model_provenance']
                if sha(ROOT/provenance[key+'_path'])!=provenance[key+'_sha256']:
                    raise RuntimeError('Transfer model asset changed after screening')
            if {row['mu'] for row in r['rows']}!=({.3} if hand else {.2,.3}) or len(r['rows'])!=(1 if hand else 2):
                raise RuntimeError('Transfer model condition missing or duplicated')
            for row in r['rows']:
                bounds=dict(self_mm=1e-6,support_mm=.15,tracking_rad=.18,range_rad=1e-6)
                if not hand:bounds['cube_mm']=1.
                if (not row['passed'] or row['stop'] or row['task_failure'] or row['done']!=len(z['goals'])
                        or row['all1ms_samples']!=len(z['goals'])*75 or row['gain']!=gain or not bounded(row['worst'],bounds)):
                    raise RuntimeError('Transfer finite condition or hand-only check failed; no writes')
                if not hand and (row['hold_samples']!=4050 or row['supported_samples']/4050<=.8
                        or row['hold_shift_mm']>=2 or row['hold_drift_deg']>=3):
                    raise RuntimeError('Bounded grasp observation lost stable support; no writes')
                if row['scope']!='Bounded supervised grasp observation; no upper-layer or next-rotation command':
                    raise RuntimeError('Observation model scope changed; no writes')
                (hands if hand else loaded).append(row)
            sources.append(dict(path=path.relative_to(ROOT).as_posix(),sha256=sha(path)))
    dependencies=['right_hand_rotation_20261109/prehardware_policy_cycle/native_cycle.py',
        'right_hand_rotation_20261109/prehardware_policy_cycle/whole_hand_support.py',
        'research_rotation_body_hold_20261104.py','hand_motor_mapping.py',
        'upper_layer_separation_certificate_20261105.py','upper_layer_cylinder_clearance_20261105.py',
        'cube_task_frame_20261105.py','right_hand_rotation_20261109/prehardware_policy_cycle/convex_pair_gap.py']
    return dict(sources=sources,model_dependency_sha256={p:sha(ROOT/p) for p in dependencies},loaded_finite_cases=len(loaded),hand_only_cases=len(hands),
        all_tested_conditions_passed=True,minimum_upper_mm=min(r['minimum_upper_mm'] for r in loaded),
        friction_hypotheses=[.2,.3],gain_hypotheses=[.5,1.,1.5],all_condition_safety_certified=False,
        initial_placement_perturbations_tested=False,physical_force_calibrated=False,
        upper_space_complete=False,full_work_grasp_task_passed=all(x['final_work_grasp_task_passed'] for x in loaded),
        intermediate90_is_diagnostic_only=True,final_left_alignment_verified=False,next_rotation_included=False)

def make_review(plan,empty,before):
    initial_scope=validate_initial_scope(empty)
    r=transport.make_review(plan,empty,transport.hashes());new=models()
    if before!=hashes():raise RuntimeError('Transfer sources changed during review')
    for key in ('loaded_model_cases','loaded_model_all_passed','loaded_model_nominal_angle_deg','loaded_geometry_cases','completed_physical_reference'):
        if key in r:r['historical_transport_'+key]=r.pop(key)
    approved=scope_approved()
    r.update(status='supervised_support_first_transfer_trial_review',code_hashes=hashes(),
        loaded_goals=len(candidate()['goals']),support_first_models=new,active_motion_limit_s=500,
        empty_initial_thumb_index_scope=initial_scope,
        existing500s_approval_retained=approved,accepted_for_operator_trial=approved,loaded_motion_authorized=approved,
        physical_transfer_success=False,physical_upper_clearance_verified=False,full_task_success_certified=False,
        intermediate90_is_diagnostic_only=True,final_left_alignment_verified=False,second_rotation_included=False,
        all_condition_safety_certified=False,no_live_policy=True,automatic_open=False,automatic_inverse=False,
        post_off_restore_policy=dict(OFF_POLICY),scope='Supervised rearward-thumb and distal-first transfer observation only; no ring closing, next rotation or left-hand operation')
    return r

def validate_review(plan,path):
    r=json.loads(Path(path).read_bytes());expected=make_review(plan,r['empty_recovery'],hashes())
    if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('Transfer proof/source/queue mismatch; no writes')
    return r

def run_screen(request,output,**kwargs):
    if not scope_approved():raise RuntimeError('Existing approved scope no longer matches; no writes')
    plan=TipFirstTrialPlan(request);ref=TipFirstTrialPlan(REQUEST);old=validate_review(ref,PROOF)
    if np.array_equal(plan.source_position(),ref.source_position()):
        r=make_review(plan,old['empty_recovery'],hashes());Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
        return validate_review(plan,output)
    process_screen(request,output,checker=ROOT/'screen_tip_first_current_20261112.py',failure_details=lambda _:'fresh support-first EMPTY recovery rejected',**kwargs)
    return validate_review(plan,output)
