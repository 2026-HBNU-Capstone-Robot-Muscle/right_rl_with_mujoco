"""Record new trial fault tests and the unchanged reporting tests."""
import json
from pathlib import Path
import time
import unittest
root=Path(__file__).resolve().parent
if __name__=='__main__':
    began=time.monotonic()
    modules=('test_cube_small_rotation_trial','test_leap_trial_summary')
    result=unittest.TextTestRunner(verbosity=1).run(unittest.defaultTestLoader.loadTestsFromNames(modules))
    report=dict(passed=result.wasSuccessful(),tests_run=result.testsRun,
                elapsed_seconds=time.monotonic()-began,modules=modules,
                failures=[(str(t),s) for t,s in result.failures],errors=[(str(t),s) for t,s in result.errors],
                hardware_commands_sent=0,real_camera_or_serial_opened=False)
    (root/'right_hand_rotation_20261022/trial_tests.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    raise SystemExit(0 if result.wasSuccessful() else 1)
