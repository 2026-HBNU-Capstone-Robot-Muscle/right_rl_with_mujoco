import contextlib,io,sys,unittest
from unittest.mock import patch
import numpy as np
import adaptive_yaw_empty_plan_20261029 as plan
import adaptive_yaw_empty_gate_20261029 as gate
import leap_adaptive_yaw_empty_20261029 as entry
from leap_yaw_load_empty_20261029 import EmptyPrefixBus
class Checks(unittest.TestCase):
    def test_original_safety_timeout_progress_and_cleanup_flow_unchanged(self):
        self.assertTrue(gate.runner_review()['all_original_health_held_progress_timeout_cleanup_code_unchanged'])
    def test_angular_command_queue_is_unchanged(self):
        class Reference(plan.Base):PLAN_VERSION=plan.AdaptiveYawEmptyPlan.PLAN_VERSION
        a=Reference(plan.REQUEST);b=plan.AdaptiveYawEmptyPlan(plan.REQUEST)
        aa=a.plan(a.source_position());bb=b.plan(b.source_position())
        self.assertEqual([p for p,q in aa],[p for p,q in bb]);np.testing.assert_array_equal([q for p,q in aa],[q for p,q in bb])
        self.assertEqual(b.ON_TIMEOUT,300);self.assertTrue(b.EMPTY_RELEASE_ONLY);self.assertIsNone(b.CAPTURE_BEFORE_PHASE)
    def test_health_and_transport_are_the_same(self):
        self.assertIs(entry.AdaptiveEmptyBus.checked_sample,EmptyPrefixBus.checked_sample)
        self.assertIs(entry.AdaptiveEmptyBus._block_once,EmptyPrefixBus._block_once)
        self.assertIs(entry.AdaptiveEmptyBus._receive,EmptyPrefixBus._receive)
    def test_current_and_stressed_fixed_hold_models_validate(self):
        p=plan.AdaptiveYawEmptyPlan(plan.REQUEST);r=gate.validate_review(p,plan.PROOF)
        self.assertEqual(len(r['empty_recovery']['static_cases']),201);self.assertEqual(len(r['empty_recovery']['dynamic_cases']),9)
        self.assertEqual(r['fixed_goal_hold_stress']['cases'],9);self.assertEqual(r['fixed_goal_hold_stress']['added_hold_s'],19.2)
        self.assertLessEqual(r['fixed_goal_hold_stress']['added_hold_s'],r['root_lag_pause_policy']['maximum_total_wait_s'])
        self.assertTrue(r['hold_counts_toward300s']);self.assertEqual(r['new_goals_or_resends_during_wait'],0)
    def test_default_entry_opens_no_devices(self):
        import cv2,dynamixel_sdk
        with patch.object(sys,'argv',['leap_adaptive_yaw_empty_20261029.py']),patch.object(cv2,'VideoCapture',side_effect=AssertionError('Camera opened')),patch.object(dynamixel_sdk,'PortHandler',side_effect=AssertionError('Motor opened')),contextlib.redirect_stdout(io.StringIO()):self.assertEqual(entry.main(),0)
if __name__=='__main__':unittest.main(verbosity=2)
