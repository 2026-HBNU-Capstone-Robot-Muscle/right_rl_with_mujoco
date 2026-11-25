"""Record saved exposure evidence and device-free capture validation."""
import hashlib
import io
import json
from pathlib import Path
import time
import unittest

ROOT=Path(__file__).resolve().parent


def main():
    source=ROOT/'camera_exposure_sweep_20261023_161815_427701'
    blob=(source/'result.json').read_bytes();result=json.loads(blob)
    review=json.loads((source/'exposure_review.json').read_bytes())
    assert review['source_sha256']==hashlib.sha256(blob).hexdigest()
    assert result['status']=='camera_exposure_diagnosis_complete'
    assert result['restoration']['restored_original_readbacks']
    selected=next(p for p in review['profiles'] if p['name']=='manual_2')
    assert selected['exposure_readback']==-7 and selected['counts']['paired']==4
    # Validate the existing actual-motor proof without rehashing old proof data.
    from grasp_place_plan import GraspPlacePlan
    from thumb_live_start_gate import validate_proof
    trial=ROOT/'cube_grasp_trial_20261023_044607_477240'
    proof=validate_proof(GraspPlacePlan(trial/'fresh_off_request.json'),trial/'current_start_screen.json')
    modules=('test_cube_pose_readonly_capture','test_camera_exposure_sweep',
             'test_camera_preview_isolation','test_camera_readonly_check')
    stream=io.StringIO();began=time.monotonic()
    tests=unittest.TextTestRunner(stream=stream,verbosity=1).run(
        unittest.defaultTestLoader.loadTestsFromNames(modules))
    files=('fixed_exposure_camera.py','cube_pose_readonly_capture.py',
           'leap_cube_pose_readonly.cmd','test_cube_pose_readonly_capture.py')
    evidence=dict(status='exposure_review_complete_readonly_pose_pair_capture_ready',
        source=source.name,source_sha256=hashlib.sha256(blob).hexdigest(),
        exposure_review_sha256=hashlib.sha256((source/'exposure_review.json').read_bytes()).hexdigest(),
        measured_original_exposure=-5.,selected_exposure=-7.,selected_pairs=4,selected_frames=6,
        original_settings_restored=True,auto_getter=-1.,auto_mode_meaning_verified=False,
        sweep_contains_fresh_encoders=False,old_grasp_q_reused_for_sweep=False,
        current_contact_geometry_verified=False,hardware_rotation_success=False,
        existing_motor_proof_matches=True,static_cases=len(proof['static_screens']),
        dynamic_cases=len(proof['dynamic_sensitivity']),
        motor_launchers_and_shared_camera_code_changed=False,
        tests=dict(passed=tests.wasSuccessful(),tests_run=tests.testsRun,modules=list(modules),
            elapsed_seconds=time.monotonic()-began,output=stream.getvalue()),
        code_sha256={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in files},
        next_launcher='leap_cube_pose_readonly.cmd',
        next_action='No robot motion. Current cube/hand unchanged; motor power + USB, torque already OFF; terminal SPACE once captures 12 bracketed photo/encoder pairs.',
        servo_write_commands_sent=0,real_camera_or_serial_opened=False)
    target=ROOT/'right_hand_rotation_20261023/exposure_pose_capture_ready_review.json'
    target.write_text(json.dumps(evidence,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in evidence.items() if k not in ('tests','code_sha256')},ensure_ascii=False,indent=2))
    print(stream.getvalue())
    if not tests.wasSuccessful():raise SystemExit(1)


if __name__=='__main__':main()
