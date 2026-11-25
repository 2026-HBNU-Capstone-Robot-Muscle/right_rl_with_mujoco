"""Combined fake-device regression and exact native proof verification."""
import json
from pathlib import Path
import time
import unittest

from validate_supported_rotation_regressions_20261023 import MODULES,EXTRA
from slow_grasp_demo_plan import SlowGraspDemoPlan
from thumb_live_start_gate import validate_proof

ROOT=Path(__file__).resolve().parent


def main():
    modules=list(dict.fromkeys(MODULES+EXTRA+('test_camera_exposure_sweep',
        'test_cube_pose_readonly_capture','test_slow_grasp_demo','test_demo_empty_start_gate')))
    began=time.monotonic()
    tests=unittest.TextTestRunner(verbosity=1).run(unittest.defaultTestLoader.loadTestsFromNames(modules))
    folder=ROOT/'right_hand_rotation_20261023/slow_grasp_demo_fixture'
    proof=validate_proof(SlowGraspDemoPlan(folder/'fresh_off_request.json'),folder/'native_screen.json')
    integration=json.loads((ROOT/'right_hand_rotation_20261022/integration_validation.json').read_bytes())
    assert integration['passed'] and len(integration['fixtures'])==21
    record=dict(passed=tests.wasSuccessful(),tests_run=tests.testsRun,modules=modules,
        failures=[(str(t),trace) for t,trace in tests.failures],errors=[(str(t),trace) for t,trace in tests.errors],
        elapsed_seconds=time.monotonic()-began,
        new_native_static_cases=len(proof['static_screens']),new_native_dynamic_cases=len(proof['dynamic_sensitivity']),
        new_native_maximum_tracking_rad=max(r['max_tracking_rad'] for r in proof['dynamic_sensitivity']),
        existing_native_fixtures_passed=len(integration['fixtures']),
        existing_fixtures_replayed=sum(not r['unchanged_validated_proof_reused'] for r in integration['fixtures']),
        hardware_commands_sent=0,real_camera_or_serial_opened=False)
    (ROOT/'right_hand_rotation_20261023/slow_grasp_demo_validation.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in record.items() if k!='modules'},indent=2))
    if not tests.wasSuccessful():raise SystemExit(1)


if __name__=='__main__':main()
