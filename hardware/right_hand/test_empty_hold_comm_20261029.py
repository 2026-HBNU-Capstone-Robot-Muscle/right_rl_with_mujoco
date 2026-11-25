import contextlib,io,json,sys,unittest
from types import SimpleNamespace
from unittest.mock import patch,Mock
import numpy as np
import empty_hold_comm_plan_20261029 as plan
import empty_hold_comm_gate_20261029 as gate
import leap_empty_hold_comm_20261029 as entry
class Clock:
    def __init__(self):self.t=0.
    def now(self):return self.t
    def sleep(self,x):self.t+=x
class Checks(unittest.TestCase):
    def test_queue_contains_no_loaded_or_cube_checkpoint(self):
        p=plan.EmptyHoldCommPlan(plan.REQUEST);rows=p.plan(p.source_position())
        self.assertIsNone(p.CAPTURE_BEFORE_PHASE);self.assertIsNone(p.CONTACT_RELEASE_BEFORE_PHASE)
        self.assertTrue(p.EMPTY_RELEASE_ONLY);self.assertFalse(any('loaded' in name or 'yaw' in name for name,_ in rows))
        self.assertLessEqual(np.max(abs(np.diff(np.vstack([p.source_position(),[q for _,q in rows]]),axis=0))),2)
    def test_runner_safety_flow_identical_except_original_guard_exposure(self):self.assertTrue(gate.runner_review()['original_safety_flow_unchanged'])
    def test_hold_paces_reads_without_catch_up_bursts(self):
        c=Clock();starts=[]
        def observe():starts.append(c.now());c.sleep(.02)
        r=entry.fixed_hold(observe,now=c.now,sleep=c.sleep,duration=.3)
        self.assertTrue(all(y-x>=.075-1e-9 for x,y in zip(starts,starts[1:])))
        self.assertEqual(r['goal_writes'],0);self.assertGreaterEqual(r['elapsed_seconds'],.3)
    def test_long_read_has_no_catch_up_burst(self):
        c=Clock();starts=[]
        def observe():starts.append(c.now());c.sleep(.12)
        entry.fixed_hold(observe,now=c.now,sleep=c.sleep,duration=.3)
        self.assertTrue(all(y-x>=.12-1e-9 for x,y in zip(starts,starts[1:])))
    def test_health_error_exits_hold_immediately(self):
        c=Clock();f=Mock(side_effect=RuntimeError('health stop'))
        with self.assertRaises(RuntimeError):entry.fixed_hold(f,now=c.now,sleep=c.sleep,duration=45)
        self.assertEqual(f.call_count,1)
    def test_missing_original_guard_prevents_hold(self):
        p=object.__new__(entry.EmptyProtocol)
        with self.assertRaises(RuntimeError):p.finish(SimpleNamespace(),np.zeros(16),{})
    def test_OFF_setting_readback_uses_full_snapshot_and_checks_torque(self):
        b=object.__new__(entry.EmptyBus);b.report={}
        values=dict(value=np.full(16,2100),_OFF_torque=np.zeros(16),_OFF_error=np.zeros(16))
        with patch.object(entry,'fast_block',return_value=(values,0,0,{})):
            v,comm,error,_=b._block_once(84,2,[('value',84,2)]);self.assertEqual(set(v),{'value'});self.assertEqual(comm,0)
        values=dict(value=np.full(16,2100),_OFF_torque=np.ones(16),_OFF_error=np.zeros(16))
        with patch.object(entry,'fast_block',return_value=(values,0,0,{})):
            with self.assertRaises(RuntimeError):b._block_once(84,2,[('value',84,2)])
    def test_goal_verification_not_redirected_to_OFF_settings(self):
        b=object.__new__(entry.EmptyBus);b.report={}
        with patch.object(entry.RetainedBus,'_block_once',return_value='NATIVE') as old,patch.object(entry,'fast_block') as fast:
            self.assertEqual(b._block_once(116,4,[]),'NATIVE');old.assert_called_once();fast.assert_not_called()
    def test_normal_health_checker_is_retained(self):
        from calibrated_two_full_health_20261028 import sample
        self.assertIs(entry.EmptyBus.checked_sample,sample)
    def test_full45s_finite_hold_is_in_review(self):
        p=plan.EmptyHoldCommPlan(plan.REQUEST);r=gate.validate_review(p,plan.PROOF)
        self.assertEqual(r['empty_recovery']['fixed_final_hold_seconds'],45.)
        self.assertTrue(r['cube_stays_OUT']);self.assertEqual(r['active_motion_limit_s'],300)
    def test_default_entry_opens_no_devices(self):
        import cv2,dynamixel_sdk
        with patch.object(sys,'argv',['leap_empty_hold_comm_20261029.py']),patch.object(cv2,'VideoCapture',side_effect=AssertionError('Camera opened')),patch.object(dynamixel_sdk,'PortHandler',side_effect=AssertionError('Motor opened')),contextlib.redirect_stdout(io.StringIO()):self.assertEqual(entry.main(),0)
if __name__=='__main__':unittest.main(verbosity=2)
