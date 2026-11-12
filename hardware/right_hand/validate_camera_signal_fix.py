"""Combined camera/control regression; no real devices."""
import json
from pathlib import Path
import time
import unittest
from validate_grasp_trial_v5 import MODULES
ROOT=Path(__file__).resolve().parent
if __name__=='__main__':
    modules=MODULES+('test_ring_release_budget','test_camera_frame_signal',
                     'test_cube_small_rotation_trial','test_leap_trial_summary',
                     'test_supported_yaw_empty')
    began=time.monotonic();result=unittest.TextTestRunner(verbosity=1).run(unittest.defaultTestLoader.loadTestsFromNames(modules))
    report=dict(passed=result.wasSuccessful(),tests_run=result.testsRun,modules=modules,
                elapsed_seconds=time.monotonic()-began,
                failures=[(str(t),s) for t,s in result.failures],errors=[(str(t),s) for t,s in result.errors],
                hardware_commands_sent=0,real_camera_or_serial_opened=False)
    (ROOT/'right_hand_rotation_20261022/camera_signal_fix_tests.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    raise SystemExit(0 if result.wasSuccessful() else 1)
