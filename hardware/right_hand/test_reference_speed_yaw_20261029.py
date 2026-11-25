import contextlib,io,json,sys,unittest
from unittest.mock import patch,Mock
import numpy as np
import reference_speed_yaw_plan_20261029 as plan
import reference_speed_yaw_gate_20261029 as gate
import leap_reference_speed_yaw_20261029 as entry
from leap_paced_retained_yaw_20261029 import PacedCapture,PacedBus
class Checks(unittest.TestCase):
    def test_all_loaded_writes_match_completed_physical_reference(self):
        r=gate.completed_reference();self.assertTrue(r['original_loaded_targets_reproduced']);self.assertEqual(r['sample_period_s'],.075)
    def test_yaw_contains638_original_targets_without_slow_repeat(self):
        yaw=[q for phase,q in plan.loaded_rows() if phase=='stage_7_horizontal_yaw']
        self.assertEqual(len(yaw),638)
    def test_current_start_and_two_tick_limit_are_screened(self):
        p=plan.ReferenceSpeedYawPlan(plan.REQUEST);r=gate.validate_review(p,plan.PROOF);rows=p.plan(p.source_position())
        self.assertEqual(r['empty_recovery']['start_ticks'],p.source_position().tolist())
        self.assertLessEqual(np.max(abs(np.diff(np.vstack([p.source_position(),[q for phase,q in rows]]),axis=0))),2)
        self.assertEqual(len(r['empty_recovery']['static_cases']),201);self.assertEqual(len(r['empty_recovery']['dynamic_cases']),9)
    def test_bus_keeps_tested_transport_and_native_health(self):
        self.assertIs(entry.ReferenceBus._block_once,PacedBus._block_once)
        self.assertIs(entry.ReferenceBus._receive,PacedBus._receive)
        self.assertIs(entry.ReferenceBus.checked_sample,PacedBus.checked_sample)
    def test_capture_keeps75ms_passive_observer(self):
        self.assertIs(entry.ReferenceCapture._observer,PacedCapture._observer)
    def test_placement_uses_original_green_up_protocol(self):
        p=object.__new__(entry.ReferenceCapture);r={}
        with patch.object(entry.NewPairGraspCapture,'place') as parent,contextlib.redirect_stdout(io.StringIO()):p.place('bus','command',r,'guard')
        parent.assert_called_once_with(p,'bus','command',r,'guard')
        self.assertEqual(r['placement_orientation'],'GREEN up; ORANGE wristward')
        self.assertNotIn('cube_placement_confirmed',r)
    def test_placement_stop_does_not_fabricate_confirmation(self):
        p=object.__new__(entry.ReferenceCapture);r={}
        with patch.object(entry.NewPairGraspCapture,'place',side_effect=RuntimeError('health stop')),contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(RuntimeError):p.place(None,None,r,None)
        self.assertNotIn('cube_placement_confirmed',r)
    def test_no_gain_or_threshold_change(self):
        r=gate.validate_review(plan.ReferenceSpeedYawPlan(plan.REQUEST),plan.PROOF)
        self.assertFalse(r['motor_gain_change']);self.assertEqual(r['current_ma'],450);self.assertEqual(r['hardware_tracking_stop_rad'],.18)
        self.assertEqual(r['active_motion_limit_s'],300);self.assertEqual(r['distinct_yaw_goal_period_s'],.075)
    def test_timing_tampering_is_rejected(self):
        p=plan.ReferenceSpeedYawPlan(plan.REQUEST);r=json.loads(plan.PROOF.read_bytes());r['distinct_yaw_goal_period_s']=.15
        native=gate.json.loads
        def changed(data,*a,**kw):
            got=native(data,*a,**kw)
            return r if isinstance(got,dict) and got.get('status')=='supervised_completed_speed_reproduction_review' else got
        with patch.object(gate.json,'loads',side_effect=changed):
            with self.assertRaises(RuntimeError):gate.validate_review(p,plan.PROOF)
    def test_reproduction_ends_at_original_open_endpoint(self):
        p=plan.ReferenceSpeedYawPlan(plan.REQUEST);rows=p.plan(p.source_position());self.assertEqual(rows[-1][1].tolist(),plan.loaded_rows()[-1][1].tolist())
        self.assertIsNotNone(p.CAPTURE_BEFORE_PHASE);self.assertFalse(getattr(p,'EMPTY_RELEASE_ONLY',False))
    def test_default_entry_opens_no_devices(self):
        import cv2,dynamixel_sdk
        with patch.object(sys,'argv',['leap_reference_speed_yaw_20261029.py']),patch.object(cv2,'VideoCapture',side_effect=AssertionError('Camera opened')),patch.object(dynamixel_sdk,'PortHandler',side_effect=AssertionError('Motor opened')),contextlib.redirect_stdout(io.StringIO()):self.assertEqual(entry.main(),0)
if __name__=='__main__':unittest.main(verbosity=2)
