"""Fake-device regression for the new loaded trial."""
import json
from pathlib import Path
import unittest
if __name__=='__main__':
    suite=unittest.defaultTestLoader.loadTestsFromName('test_supported_cube_rotation')
    result=unittest.TextTestRunner(verbosity=1).run(suite)
    record=dict(passed=result.wasSuccessful(),tests_run=result.testsRun,
        failures=[(str(t),trace) for t,trace in result.failures],errors=[(str(t),trace) for t,trace in result.errors],
        hardware_commands_sent=0,real_camera_or_serial_opened=False)
    target=Path(__file__).resolve().parent/'right_hand_rotation_20261023/loaded_control_tests.json'
    target.write_text(json.dumps(record,indent=2),encoding='utf-8')
    raise SystemExit(0 if result.wasSuccessful() else 1)
