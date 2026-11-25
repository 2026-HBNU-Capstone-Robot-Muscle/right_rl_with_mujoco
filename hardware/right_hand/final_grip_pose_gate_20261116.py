"""Source-bound EMPTY pose review; legacy transport settings remain unchanged."""
import json
from pathlib import Path
import numpy as np
import four_reply_yaw_gate_20261029 as transport
from calibrated_quarter_gate_20261028 import bounded
from interrupted_finger_scope_20261116 import validate_report
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen
from verified_off_restore_context_20261105 import POLICY as OFF_POLICY
from final_grip_pose_plan_20261116 import ROOT, OUT, REQUEST, PROOF, CANDIDATE, FinalGripPosePlan, hashes, candidate, sha
from book_empty_preparation_plan import FRONTS, offsets

def full_review(plan, full):
    rows = plan.plan(plan.source_position())
    import hashlib
    digest = hashlib.sha256(json.dumps([(p,q.tolist()) for p,q in rows], separators=(',',':')).encode()).hexdigest()
    if (full.get('all_passed') is not True or full.get('start_ticks') != plan.source_position().tolist()
            or full.get('prefix_sha256') != digest or full.get('goals') != len(rows)):
        raise RuntimeError('Complete EMPTY pose path has not passed its own model screen')
    static, dynamic = full.get('static_cases', []), full.get('dynamic_cases', [])
    if (len(static) != 201 or len(dynamic) != 9
            or {(r['book_front_mm'],tuple(r['offset_ticks'])) for r in static}
               != {(f,tuple(o)) for f in FRONTS for o in offsets()}
            or {(r['book_front_mm'],r['gain_scale']) for r in dynamic}
               != {(f,g) for f in FRONTS for g in (.5,1.,1.5)}):
        raise RuntimeError('Missing or duplicate complete-pose model cases')
    for r in static:
        if not r.get('passed') or not bounded(r, dict(max_self_mm=1e-6,max_support_after_release_mm=.15,
                initial_support_increase_mm=1e-6,step_support_increase_mm=1e-6)):
            raise RuntimeError('Complete EMPTY static pose case rejected')
    for r in dynamic:
        if not r.get('passed') or not bounded(r, dict(max_self_mm=1e-6,max_support_mm=.15,
                max_tracking_rad=.16,post_release_tracking_rad=.12,unapproved_range_excess_rad=1e-6,
                initial_support_increase_mm=1e-6,root_clear_seconds=3.575)):
            raise RuntimeError('Complete EMPTY dynamic pose case rejected')
    validate_report(full)
    return full

def make_review(plan, empty, full, before):
    scope = validate_report(empty)
    full_review(plan, full)
    guards = json.loads((OUT/'runtime_guard_review.json').read_bytes())
    if guards.get('candidate_sha256') != sha(CANDIDATE):
        raise RuntimeError('Runtime guard replay belongs to a different pose route')
    required = {('whole_path',1.),('whole_path',1.5),('new_pose_segment',.5)}
    if not required <= {(v['scope'],v['gain']) for v in guards['cases'] if v.get('passed') and v.get('done') == v.get('expected')}:
        raise RuntimeError('Nominal whole path or weak-drive new pose guard replay failed')
    r = transport.make_review(plan, empty, transport.hashes())
    if before != hashes():
        raise RuntimeError('Pose sources changed during screen')
    # Legacy loaded evidence belongs to the inherited transport, not this pose.
    for key in tuple(r):
        if key.startswith(('loaded_', 'model_yaw', 'nominal_model_yaw', 'completed_physical')):
            r['historical_transport_'+key] = r.pop(key)
    r.update(status='empty_final_grip_pose_ready_for_operator', code_hashes=hashes(),
        full_empty_pose=full, empty_initial_thumb_index_scope=scope,
        empty_only=True, cube_placement_allowed=False, rotation_included=False,
        physical_cube_grasp_verified=False, final_left_clearance_verified=False,
        automatic_open=False, no_live_policy=True, active_motion_limit_s=500,
        post_off_restore_policy=dict(OFF_POLICY),
        full_screen_checker_sha256=sha(ROOT/'screen_final_grip_pose_full_20261116.py'),
        reference_runtime_guard_replay=guards,
        all_tested_runtime_conditions_passed=guards['passed'],
        weak_drive_whole_start_guard_passed=next(v['passed'] for v in guards['cases'] if (v['scope'],v['gain'])==('whole_path',.5)),
        scope='EMPTY hand shape inspection only; explicit SPACE exit or existing total500s bound')
    from final_pose_temporary_support_20261116 import POLICY
    r['temporary_support_assumption']=dict(POLICY)
    r['temporary_support_dimensions_verified']=False
    r['passive_root_start_lower_ticks']={'8':1814,'12':1819,'16':1838}
    r['passive_root_model_soft_limit_residual_ticks']=2
    r['passive_root_model_soft_limit_residual_scope']='Simulation only; initial_hold/initial reentry only. No additional goal/admission/health slack.'
    from final_grip_standard_read_20261116 import POLICY as READ_POLICY
    r['active_health_read_policy']=dict(READ_POLICY)
    r['final_grip_standard_read_live_ON_verified']=False
    return r

def validate_review(plan, path):
    r = json.loads(Path(path).read_bytes())
    expected = make_review(plan, r['empty_recovery'], r['full_empty_pose'], hashes())
    if any(r.get(k) != v for k,v in expected.items()):
        raise RuntimeError('Final pose proof/source/command queue mismatch; no writes')
    return r

def run_screen(request, output, **kwargs):
    plan, reference = FinalGripPosePlan(request), FinalGripPosePlan(REQUEST)
    old = validate_review(reference, PROOF)
    if np.array_equal(plan.source_position(), reference.source_position()):
        r = make_review(plan, old['empty_recovery'], old['full_empty_pose'], hashes())
        Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
    else:
        process_screen(request, output, checker=ROOT/'screen_final_grip_pose_current_20261116.py',
            failure_details=lambda _: 'current OFF -> final EMPTY pose rejected', **kwargs)
    return validate_review(plan, output)
