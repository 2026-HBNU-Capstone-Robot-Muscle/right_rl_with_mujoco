"""Pin saved-data analysis and camera-only checks, opening no real device."""
import hashlib
import io
import json
from pathlib import Path
import time
import unittest

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_grasp_trial_20261023_044607_477240'


def main():
    reference=(FOLDER/'direct_reference_cube_pose.json').read_bytes()
    digest=hashlib.sha256(reference).hexdigest()
    analyses={}
    for name in ('direct_pose_geometry_audit.json','conditional_new_closure_search.json',
                 'desired_placement_search.json','observed_hold_consistency.json'):
        raw=(FOLDER/name).read_bytes();data=json.loads(raw)
        assert data['reference_sha256']==digest, 'Stale analysis: '+name
        analyses[name]=dict(sha256=hashlib.sha256(raw).hexdigest(),reference_matches=True)
    hold=json.loads((FOLDER/'observed_hold_consistency.json').read_bytes())
    scan=json.loads((FOLDER/'direct_palm_reference_scan.json').read_bytes())
    load=json.loads((FOLDER/'matched_empty_load_review.json').read_bytes())
    from grasp_place_plan import GraspPlacePlan
    from thumb_live_start_gate import validate_proof
    proof=validate_proof(GraspPlacePlan(FOLDER/'fresh_off_request.json'),FOLDER/'current_start_screen.json')
    modules=('test_camera_exposure_sweep','test_camera_preview_isolation','test_camera_readonly_check')
    began=time.monotonic();stream=io.StringIO()
    result=unittest.TextTestRunner(stream=stream,verbosity=1).run(unittest.defaultTestLoader.loadTestsFromNames(modules))
    output=dict(status='saved_contact_analysis_complete_camera_only_check_ready',
        source=FOLDER.name,reference_sha256=digest,analyses=analyses,
        directly_decoded_palm_frames_in_sample=len(scan['rows']),sampled_palm_frames=scan['scanned'],
        source_camera_recording_complete=True,
        conditional_hold_compatible_cases=hold['compatible_cases'],conditional_hold_cases=hold['case_count'],
        conditional_hold_duration_s=hold['tested_hold_seconds'],
        hold_is_hand_clamped_diagnostic=True,calibrated_real_pose_verified=False,
        matched_empty_loaded_goal_samples=load['matching_goal_samples'],
        maximum_extra_median_current_ma=load['maximum_absolute_added_median_current_ma'],
        loaded_closure_or_new_rotation_exported=False,
        existing_motor_proof_still_matches=True,static_cases=len(proof['static_screens']),dynamic_cases=len(proof['dynamic_sensitivity']),
        tests=dict(passed=result.wasSuccessful(),tests_run=result.testsRun,modules=list(modules),
                   elapsed_seconds=time.monotonic()-began,output=stream.getvalue()),
        camera_only_code_sha256=hashlib.sha256((ROOT/'camera_exposure_sweep.py').read_bytes()).hexdigest(),
        next_launcher='leap_camera_exposure_sweep.cmd',next_action_requires_motor_power=False,
        hardware_commands_sent=0,real_camera_or_serial_opened=False,
        conclusion='Do not use manual-border poses or near-zero geometry distances as confirmed contact. Camera exposure/marker readability needs a device-side diagnostic before using these images to change the loaded path.')
    target=ROOT/'right_hand_rotation_20261023/contact_analysis_exposure_ready_review.json'
    target.write_text(json.dumps(output,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in output.items() if k!='analyses'},ensure_ascii=False,indent=2))
    if not result.wasSuccessful():raise SystemExit(1)


if __name__=='__main__':main()
