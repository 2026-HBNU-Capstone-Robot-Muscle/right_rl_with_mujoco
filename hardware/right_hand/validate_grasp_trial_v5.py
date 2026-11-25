"""Record the complete offline regression suite; never opens real devices."""
import datetime as dt
import json
from pathlib import Path
import time
import unittest

ROOT=Path(__file__).resolve().parent
MODULES=(
    'test_leap_thumb_prefix_empty','test_empty_trial_camera','test_thumb_continuation_empty',
    'test_thumb_current_start','test_hand_preparation_timing','test_prepared_placement_capture',
    'test_sequence_voltage','test_grasp_approach_empty','test_held_joint_guard',
    'test_grasp_place_trial','test_initial_contact_escape','test_screen_failure_diagnostics',
    'test_goal_readback_confirmation')

if __name__=='__main__':
    suite=unittest.defaultTestLoader.loadTestsFromNames(MODULES)
    began=time.perf_counter()
    result=unittest.TextTestRunner(verbosity=1).run(suite)
    report=dict(timestamp=dt.datetime.now().astimezone().isoformat(),
        passed=result.wasSuccessful(),tests_run=result.testsRun,elapsed_seconds=time.perf_counter()-began,
        failures=[(str(t),trace) for t,trace in result.failures],
        errors=[(str(t),trace) for t,trace in result.errors],modules=MODULES,
        hardware_commands_sent=0,real_camera_or_serial_opened=False)
    (ROOT/'prepared_cube_review_20261022/grasp_trial_v5_tests.json').write_text(
        json.dumps(report,indent=2),encoding='utf-8')
    raise SystemExit(0 if result.wasSuccessful() else 1)
