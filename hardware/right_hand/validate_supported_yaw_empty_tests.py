import json
from pathlib import Path
import unittest
ROOT=Path(__file__).resolve().parent
if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=1).run(unittest.defaultTestLoader.loadTestsFromName('test_supported_yaw_empty'))
    report=dict(passed=result.wasSuccessful(),tests_run=result.testsRun,
                failures=[(str(t),s) for t,s in result.failures],errors=[(str(t),s) for t,s in result.errors],
                hardware_commands_sent=0,real_camera_or_serial_opened=False)
    (ROOT/'right_hand_rotation_20261022/supported_empty_tests.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    raise SystemExit(0 if result.wasSuccessful() else 1)
