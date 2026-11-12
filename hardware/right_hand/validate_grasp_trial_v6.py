"""Run the offline regressions for the unchanged v6 motion path."""
import datetime as dt
import json
from pathlib import Path
import time
import unittest
from validate_grasp_trial_v5 import MODULES

ROOT=Path(__file__).resolve().parent
if __name__=='__main__':
    names=MODULES+('test_ring_release_budget',)
    began=time.perf_counter()
    result=unittest.TextTestRunner(verbosity=1).run(unittest.defaultTestLoader.loadTestsFromNames(names))
    report=dict(timestamp=dt.datetime.now().astimezone().isoformat(),passed=result.wasSuccessful(),
        tests_run=result.testsRun,elapsed_seconds=time.perf_counter()-began,
        failures=[(str(t),trace) for t,trace in result.failures],
        errors=[(str(t),trace) for t,trace in result.errors],modules=names,
        hardware_commands_sent=0,real_camera_or_serial_opened=False)
    (ROOT/'prepared_cube_review_20261022/grasp_trial_v6_tests.json').write_text(
        json.dumps(report,indent=2),encoding='utf-8')
    raise SystemExit(0 if result.wasSuccessful() else 1)
