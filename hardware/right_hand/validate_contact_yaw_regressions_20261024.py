"""Fault-injected regression plus seven exact native proofs; no real devices."""
import json
from pathlib import Path
import time
import unittest
from validate_supported_rotation_regressions_20261023 import MODULES,EXTRA
from grasped_yaw_plan import GraspedYawPlan,CANDIDATE_SHA
from thumb_live_start_gate import validate_proof
from revalidate_contact_yaw_20261024 import OLD_FIXTURES,LATEST

ROOT=Path(__file__).resolve().parent


def main():
    modules=list(dict.fromkeys(MODULES+EXTRA+('test_camera_exposure_sweep','test_cube_pose_readonly_capture',
        'test_slow_grasp_demo','test_demo_empty_start_gate','test_grasped_yaw_trial','test_yaw_camera_profile',
        'test_read_recovery_confirmation')))
    began=time.monotonic()
    tests=unittest.TextTestRunner(verbosity=1).run(unittest.defaultTestLoader.loadTestsFromNames(modules))
    records=[]
    for folder in OLD_FIXTURES+(LATEST,):
        proof=validate_proof(GraspedYawPlan(folder/'fresh_off_request.json'),folder/'native_screen.json')
        records.append(dict(fixture=str(folder.relative_to(ROOT)),accepted=proof['accepted'],
            static_cases=len(proof['static_screens']),dynamic_cases=len(proof['dynamic_sensitivity']),
            maximum_tracking_rad=max(r['max_tracking_rad'] for r in proof['dynamic_sensitivity'])))
    previous=json.loads((ROOT/'right_hand_rotation_20261022/integration_validation.json').read_bytes())
    assert previous['passed'] and len(previous['fixtures'])==21
    result=dict(passed=tests.wasSuccessful(),tests_run=tests.testsRun,
        failures=[(str(t),trace) for t,trace in tests.failures],errors=[(str(t),trace) for t,trace in tests.errors],
        native_fixtures=records,candidate_sha256=CANDIDATE_SHA,
        previous_native_fixtures_passed=21,legacy_programs_changed=False,
        elapsed_s=time.monotonic()-began,hardware_commands_sent=0,real_devices_opened=False)
    (ROOT/'right_hand_rotation_20261024/contact_yaw_regression_validation.json').write_text(
        json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))
    if not tests.wasSuccessful():raise SystemExit(1)


if __name__=='__main__':main()
