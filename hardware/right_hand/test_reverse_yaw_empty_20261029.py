import contextlib,copy,io,json,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import reverse_yaw_empty_plan_20261029 as p
import reverse_yaw_empty_gate_20261029 as gate
import leap_reverse_yaw_empty_20261029 as entry
from leap_four_reply_yaw_20261029 import FourReplyBus
from hand_motor_mapping import SIM_TO_MOTOR_IDS
class Checks(unittest.TestCase):
    def test_entire_original_cycle_is_reversed_without_new_pose(self):
        r=gate.reverse_source();self.assertTrue(r['exact_reverse_order']);self.assertTrue(r['all_commanded_poses_already_in_original_geometry'])
        old=np.load(p.LOADED_PATH);new=np.load(p.HAND_PATH)
        np.testing.assert_array_equal(new['goals'][new['phases']=='stage_7_horizontal_yaw'],old['goals'][old['phases']=='yaw'][::-1])
    def test_native_ranges_step_and_total_time(self):
        from loaded_quarter_reentry_20261025 import limits
        plan=p.ReverseYawEmptyPlan(p.REQUEST);start=plan.source_position();a=np.asarray([q for phase,q in plan.plan(start)]);lo,hi=limits()
        self.assertTrue(np.all(a>=lo) and np.all(a<=hi));self.assertLessEqual(int(max(abs(np.diff(np.vstack([start,a]),axis=0)).ravel())),2)
        self.assertLessEqual((len(a)+40)*.075+20,300)
    def test_empty_only_and_one_space_protocol(self):
        plan=p.ReverseYawEmptyPlan(p.REQUEST);self.assertTrue(plan.EMPTY_RELEASE_ONLY);self.assertIsNone(plan.CAPTURE_BEFORE_PHASE)
        self.assertIsNone(plan.CONTACT_RELEASE_BEFORE_PHASE)
        from leap_yaw_load_empty_20261029 import EmptyPrefixProtocol
        self.assertIs(plan.CAPTURE_PROTOCOL_TYPE,EmptyPrefixProtocol)
        self.assertEqual(tuple(SIM_TO_MOTOR_IDS),(8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1))
    def test_current_prefix_and_reverse_finite_models_complete(self):
        r=gate.validate_review(p.ReverseYawEmptyPlan(p.REQUEST),p.PROOF)
        self.assertEqual(len(r['empty_recovery']['static_cases']),201);self.assertEqual(len(r['empty_recovery']['dynamic_cases']),9)
        self.assertEqual(r['reverse_empty_hand_dynamics']['cases'],9);self.assertTrue(r['reverse_empty_hand_dynamics']['cube_disabled'])
        self.assertFalse(r['inverse_cube_trial_ready']);self.assertFalse(r['physical_inverse_rotation_verified'])
    def test_bus_write_and_health_are_unchanged(self):
        for name in ('write','checked_sample','_receive','_block_once'):self.assertIs(getattr(entry.ReverseEmptyBus,name),getattr(FourReplyBus,name))
        self.assertIs(entry.confirmed_run, __import__('horizontal_quarter_motion_runner_20261027').run)
    def test_changed_goal_digest_is_rejected(self):
        r=json.loads(p.PROOF.read_bytes());r['plan_sha256']='0'*64
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'bad.json';path.write_text(json.dumps(r),encoding='utf-8')
            with self.assertRaises(RuntimeError):gate.validate_review(p.ReverseYawEmptyPlan(p.REQUEST),path)
    def test_default_entry_opens_no_devices(self):
        import cv2,dynamixel_sdk
        with patch.object(sys,'argv',['leap_reverse_yaw_empty_20261029.py']),patch.object(cv2,'VideoCapture',side_effect=AssertionError('Camera opened')),patch.object(dynamixel_sdk,'PortHandler',side_effect=AssertionError('Motor opened')),contextlib.redirect_stdout(io.StringIO()):self.assertEqual(entry.main(),0)
if __name__=='__main__':unittest.main(verbosity=2)
