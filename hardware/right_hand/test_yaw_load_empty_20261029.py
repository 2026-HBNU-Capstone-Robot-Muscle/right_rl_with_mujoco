import contextlib,io,sys,unittest
from unittest.mock import patch
import numpy as np
import yaw_load_empty_plan_20261029 as plan
import yaw_load_empty_gate_20261029 as gate
import leap_yaw_load_empty_20261029 as entry
from calibrated_quarter_plan_20261028 import loaded_rows
from leap_four_reply_yaw_20261029 import FourReplyBus
class Checks(unittest.TestCase):
    def test_diagnostic_uses_exact_original_prefix_through_yaw320(self):
        rows=plan.motion_prefix();source=loaded_rows()[:len(rows)]
        self.assertEqual([p for p,q in rows],[p for p,q in source]);np.testing.assert_array_equal([q for p,q in rows],[q for p,q in source])
        self.assertEqual(sum(p=='stage_7_horizontal_yaw' for p,q in rows),320)
        self.assertGreater(320,267);self.assertFalse(any(p=='stage_9_loaded_open' for p,q in rows))
    def test_no_cube_checkpoints_or_secondary_SPACE(self):
        p=plan.YawLoadEmptyPlan(plan.REQUEST)
        self.assertTrue(p.CAPTURE_PLACEMENT);self.assertTrue(p.EMPTY_RELEASE_ONLY)
        self.assertIsNone(p.CAPTURE_BEFORE_PHASE);self.assertIsNone(p.CONTACT_RELEASE_BEFORE_PHASE)
        self.assertTrue(p.CAMERA_TYPE.EMPTY_ONLY)
    def test_finish_creates_no_fake_cube_placement_or_removal(self):
        protocol=object.__new__(entry.EmptyPrefixProtocol);report={}
        with contextlib.redirect_stdout(io.StringIO()):protocol.finish(None,np.zeros(16),report)
        self.assertTrue(report['empty_reference_prefix_complete']);self.assertFalse(report['cube_present'])
        self.assertNotIn('cube_placement_confirmed',report);self.assertNotIn('cube_removal_confirmed',report)
    def test_unexpected_release_is_rejected(self):
        with self.assertRaises(RuntimeError):object.__new__(entry.EmptyPrefixProtocol).release()
    def test_native_transport_health_and_stop_logic_are_inherited(self):
        self.assertIs(entry.EmptyPrefixBus._block_once,FourReplyBus._block_once)
        self.assertIs(entry.EmptyPrefixBus.checked_sample,FourReplyBus.checked_sample)
        self.assertIs(entry.EmptyPrefixBus._receive,FourReplyBus._receive)
    def test_current_recovery_and_hand_dynamics_both_validate(self):
        p=plan.YawLoadEmptyPlan(plan.REQUEST);r=gate.validate_review(p,plan.PROOF)
        self.assertEqual(len(r['empty_recovery']['static_cases']),201);self.assertEqual(len(r['empty_recovery']['dynamic_cases']),9)
        self.assertEqual(r['empty_hand_prefix_dynamics']['cases'],9);self.assertTrue(r['empty_hand_prefix_dynamics']['cube_disabled'])
        self.assertTrue(r['cube_stays_OUT']);self.assertEqual(r['final_fixed_hold_seconds'],1.5)
    def test_trajectory_never_exceeds_original_two_tick_and_travel_limits(self):
        p=plan.YawLoadEmptyPlan(plan.REQUEST);rows=p.plan(p.source_position());values=np.vstack([p.source_position(),[q for phase,q in rows]])
        self.assertLessEqual(np.max(abs(np.diff(values,axis=0))),2)
        self.assertLessEqual(np.max(abs(values[:,:4]-values[0,:4])),p.MAX_THUMB_TRAVEL)
        self.assertEqual(p.SAMPLE_PERIOD,.075);self.assertEqual(p.ON_TIMEOUT,300)
    def test_default_entry_opens_no_devices(self):
        import cv2,dynamixel_sdk
        with patch.object(sys,'argv',['leap_yaw_load_empty_20261029.py']),patch.object(cv2,'VideoCapture',side_effect=AssertionError('Camera opened')),patch.object(dynamixel_sdk,'PortHandler',side_effect=AssertionError('Motor opened')),contextlib.redirect_stdout(io.StringIO()):self.assertEqual(entry.main(),0)
if __name__=='__main__':unittest.main(verbosity=2)
