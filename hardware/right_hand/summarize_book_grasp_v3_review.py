"""Saved evidence review only; current physical posture is not approved."""
import hashlib
import json
from pathlib import Path
from book_grasp_trial_plan import ROOT,BookGraspPlan,reached_prefix
from book_grasp_trial_gate import validate_proof
from leap_book_cube_grasp_trial import require_offline_review

folder=ROOT/'right_hand_rotation_20261024/book20_current_review/grasp_route_fixture_v3'
proof=require_offline_review();plan=BookGraspPlan(folder/'request.json')
camera_folder=ROOT/'book_camera_confirmation_20261024_173636_997894'
camera=json.loads((camera_folder/'result.json').read_bytes())
read_folder=ROOT/'book_settled_off_readonly_20261024_173647_632324'
read=json.loads((read_folder/'result.json').read_bytes())
validation=json.loads((folder/'device_free_validation.json').read_bytes())
r=dict(implementation_reviewed=True,current_physical_posture_approved=False,
    reason='Latest read-only pose has ID4 below native range, and latest camera shows cube still on hand. Need cube removal before a new EMPTY acquisition.',
    plan_version=plan.PLAN_VERSION,native_proof_sha256=hashlib.sha256((folder/'screen.json').read_bytes()).hexdigest(),
    native_static_cases=len(proof['static_screens']),native_dynamic_cases=len(proof['dynamic_sensitivity']),
    original_loaded_targets=322,shortened_loaded_targets=len(reached_prefix()),
    max_model_tracking_rad=max(s['max_tracking_rad'] for s in proof['dynamic_sensitivity']),
    max_model_tracking_after_clearance_rad=max(s['max_tracking_after_clearance_rad'] for s in proof['dynamic_sensitivity']),
    endpoint_ticks=list(reached_prefix()[-1]),
    original_actual_tracking_trip_rad=.18,original_actual_goal_current_ma=450,
    original_actual_no_progress_guard_unchanged=True,
    camera_stability=dict(folder=str(camera_folder),duration_seconds=camera['duration_seconds'],
        frames=camera['camera_recording']['frame_count'],status=camera['camera_recording']['status'],
        exposure_restoration_verified=camera['camera_recording']['exposure_restoration_verified'],
        transient_exposure_mismatch_reproduced=False),
    camera_confirmation_policy='Opt-in only: at most 3 fresh extra READs / 100ms, two consecutive matches, at most 3 mismatch events / 2s. No setter retry. Persistent mismatch still stops.',
    camera_backend_reference='https://raw.githubusercontent.com/opencv/opencv/4.x/modules/videoio/src/cap_dshow.cpp',
    backend_reference_note='DSHOW getter can return -1 when unavailable. This does not prove the observed hardware mismatch was transient. Fake-worker tests inject and confirm the transient case.',
    device_free_validation=validation,
    actual_latest_readonly=dict(folder=str(read_folder),position=read['preflight_state']['position'],
        servo_write_commands_sent=0,torque_all_OFF=not any(read['preflight_state']['torque'])),
    assistant_motor_write_commands_sent=0,physical_grasp_verified=False,physical_rotation_verified=False,
    next_user_action='Remove cube to table, leave fingers unchanged, and say removed. Do not prescribe a new motor trial before reviewing the new EMPTY pose.')
(folder/'readiness_review.json').write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in r.items() if k not in ('actual_latest_readonly','device_free_validation')},ensure_ascii=False))
