"""Record the real device-free suite result for the current production route."""
import json
from pathlib import Path
import unittest
names=('test_book_dynamic_yaw','test_book_grasp_trial','test_observed_book_model_policy',
    'test_preserve_exposure_camera','test_prepared_placement_capture',
    'test_goal_readback_confirmation','test_read_recovery_confirmation','test_held_joint_guard',
    'test_empty_trial_camera','test_demo_empty_start_gate','test_camera_property_confirmation','test_yaw_camera_profile')
result=unittest.TextTestRunner(verbosity=1).run(unittest.defaultTestLoader.loadTestsFromNames(names))
record=dict(modules=list(names),tests_run=result.testsRun,failures=len(result.failures),errors=len(result.errors),
    skipped=len(result.skipped),successful=result.wasSuccessful(),motor_or_camera_devices_opened=False,
    old_v1_entry_redirected_to_v2=True,operator_SPACE_not_physical_removal_proof=True)
folder=Path(__file__).resolve().parent/'right_hand_rotation_20261024/book20_current_review/dynamic_yaw_fixture_v2'
(folder/'device_free_validation.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
raise SystemExit(0 if result.wasSuccessful() else 1)
