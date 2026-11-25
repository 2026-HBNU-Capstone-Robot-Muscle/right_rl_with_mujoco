"""Existing device-free regressions plus new small-turn native evidence."""
import json
from pathlib import Path
import time
import unittest
from validate_supported_rotation_regressions_20261023 import MODULES,EXTRA
from grasped_yaw_plan import GraspedYawPlan
from thumb_live_start_gate import validate_proof

ROOT=Path(__file__).resolve().parent


def main():
    modules=list(dict.fromkeys(MODULES+EXTRA+('test_camera_exposure_sweep','test_cube_pose_readonly_capture',
        'test_slow_grasp_demo','test_demo_empty_start_gate','test_grasped_yaw_trial','test_yaw_camera_profile',
        'test_read_recovery_confirmation')))
    start=time.monotonic();tests=unittest.TextTestRunner(verbosity=1).run(unittest.defaultTestLoader.loadTestsFromNames(modules))
    records=[]
    for label in ('grasped_yaw_fixture','grasped_yaw_prior_fixture','grasped_yaw_open_fixture',
                  'grasped_yaw_after_stop_fixture','grasped_yaw_goal_stop_fixture',
                  'grasped_yaw_LED_complete_fixture'):
        folder=ROOT/'right_hand_rotation_20261023'/label
        proof=validate_proof(GraspedYawPlan(folder/'fresh_off_request.json'),folder/'native_screen.json')
        records.append(dict(fixture=label,static_cases=len(proof['static_screens']),
            dynamic_cases=len(proof['dynamic_sensitivity']),maximum_tracking_rad=max(r['max_tracking_rad'] for r in proof['dynamic_sensitivity'])))
    old=json.loads((ROOT/'right_hand_rotation_20261022/integration_validation.json').read_bytes())
    assert old['passed'] and len(old['fixtures'])==21
    result=dict(passed=tests.wasSuccessful(),tests_run=tests.testsRun,
        failures=[(str(t),trace) for t,trace in tests.failures],errors=[(str(t),trace) for t,trace in tests.errors],
        native_fixtures=records,previous_native_fixtures_passed=21,
        previous_native_fixtures_replayed=sum(not r['unchanged_validated_proof_reused'] for r in old['fixtures']),
        elapsed_s=time.monotonic()-start,hardware_commands_sent=0,real_devices_opened=False)
    (ROOT/'right_hand_rotation_20261023/grasped_yaw_validation.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))
    if not tests.wasSuccessful():raise SystemExit(1)


if __name__=='__main__':main()
