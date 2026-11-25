"""Combined regression after adding the loaded observation. Fake devices only."""
import json
from pathlib import Path
import time
import unittest
from validate_grasp_trial_v5 import MODULES
EXTRA=('test_ring_release_budget','test_camera_frame_signal','test_cube_small_rotation_trial',
       'test_leap_trial_summary','test_supported_yaw_empty','test_supported_cube_rotation',
       'test_camera_preview_isolation','test_camera_readonly_check')
if __name__=='__main__':
    began=time.monotonic();modules=MODULES+EXTRA
    result=unittest.TextTestRunner(verbosity=1).run(unittest.defaultTestLoader.loadTestsFromNames(modules))
    report=dict(passed=result.wasSuccessful(),tests_run=result.testsRun,modules=modules,
        elapsed_seconds=time.monotonic()-began,
        failures=[(str(t),trace) for t,trace in result.failures],errors=[(str(t),trace) for t,trace in result.errors],
        hardware_commands_sent=0,real_camera_or_serial_opened=False)
    target=Path(__file__).resolve().parent/'right_hand_rotation_20261023/regression_tests.json'
    target.write_text(json.dumps(report,indent=2),encoding='utf-8')
    raise SystemExit(0 if result.wasSuccessful() else 1)
