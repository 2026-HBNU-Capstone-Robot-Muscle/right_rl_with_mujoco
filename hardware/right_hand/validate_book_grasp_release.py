"""Run the relevant device-free suite and preserve its actual result."""
import json
from pathlib import Path
import unittest

names=('test_book_grasp_trial','test_observed_book_model_policy','test_preserve_exposure_camera','test_prepared_placement_capture',
    'test_goal_readback_confirmation','test_read_recovery_confirmation',
    'test_held_joint_guard','test_empty_trial_camera','test_demo_empty_start_gate',
    'test_camera_property_confirmation','test_yaw_camera_profile')
suite=unittest.defaultTestLoader.loadTestsFromNames(names)
result=unittest.TextTestRunner(verbosity=1).run(suite)
record=dict(modules=list(names),tests_run=result.testsRun,failures=len(result.failures),
    errors=len(result.errors),skipped=len(result.skipped),successful=result.wasSuccessful(),
    motor_or_camera_devices_opened=False,
    note='Real native proof fixture plus fake bus/clock and camera checks. No hardware execution.')
path=Path(__file__).resolve().parent/'right_hand_rotation_20261024/book20_current_review/grasp_route_fixture_v7/device_free_validation.json'
path.write_text(json.dumps(record,indent=2),encoding='utf-8')
raise SystemExit(0 if result.wasSuccessful() else 1)
