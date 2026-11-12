import contextlib,io,json,sys,unittest
from unittest.mock import Mock,patch
from types import SimpleNamespace
import numpy as np
import paced_retained_yaw_plan_20261029 as plan
import paced_retained_yaw_gate_20261029 as gate
import leap_paced_retained_yaw_20261029 as entry
from leap_empty_hold_comm_20261029 import EmptyBus
class Clock:
    def __init__(self):self.t=0.
    def now(self):return self.t
    def sleep(self,x):self.t+=x
class Checks(unittest.TestCase):
    def test_fast_passive_reads_are_spaced_at_least75ms(self):
        c=Clock();starts=[]
        def observe(phase):starts.append(c.now());c.sleep(.015);return phase
        p=entry.pace_observer(observe,lambda:None,now=c.now,sleep=c.sleep)
        for _ in range(8):self.assertEqual(p('place_cube'),'place_cube');c.sleep(.02)
        self.assertTrue(all(y-x>=.075-1e-9 for x,y in zip(starts,starts[1:])))
        self.assertAlmostEqual(starts[1],.075)
    def test_processing_delay_does_not_produce_catch_up_burst(self):
        c=Clock();starts=[]
        def observe(phase):starts.append(c.now());c.sleep(.15 if len(starts)==1 else .015)
        p=entry.pace_observer(observe,lambda:None,now=c.now,sleep=c.sleep)
        for _ in range(4):p('loaded_countdown')
        np.testing.assert_allclose(starts,[0.,.15,.225,.3])
    def test_cancel_before_wait_does_not_read_health(self):
        f=Mock();p=entry.pace_observer(f,Mock(side_effect=RuntimeError('Q')))
        with self.assertRaises(RuntimeError):p('place_cube')
        f.assert_not_called()
    def test_cancel_after_wait_prevents_second_health_read(self):
        c=Clock();f=Mock(side_effect=lambda phase:c.sleep(.015));checks=Mock(side_effect=[None,None,None,RuntimeError('Q')])
        p=entry.pace_observer(f,checks,now=c.now,sleep=c.sleep);p('place_cube')
        with self.assertRaises(RuntimeError):p('place_cube')
        self.assertEqual(f.call_count,1)
    def test_health_error_is_not_swallowed_or_retried(self):
        f=Mock(side_effect=RuntimeError('health stop'));p=entry.pace_observer(f,lambda:None)
        with self.assertRaises(RuntimeError):p('remove_cube')
        self.assertEqual(f.call_count,1)
    def test_native_sampler_and_held_guard_are_invoked_by_observer(self):
        p=object.__new__(entry.PacedCapture);p.cancel=Mock();p.sleep=lambda _:None
        state={'position':np.zeros(16)};guard=SimpleNamespace(check=Mock());report={}
        def checked(bus,command,r,phase,snapshot_check):snapshot_check(state);return state,None
        b=SimpleNamespace(checked_sample=Mock(side_effect=checked))
        observe=p._observer(b,np.zeros(16),report,guard)
        self.assertIs(observe('place_cube'),state);b.checked_sample.assert_called_once();guard.check.assert_called_once()
    def test_tested_OFF_and_native_health_implementations_are_reused_exactly(self):
        self.assertIs(entry.PacedBus._block_once,EmptyBus._block_once)
        self.assertIs(entry.PacedBus._receive,EmptyBus._receive)
        self.assertIs(entry.PacedBus.checked_sample,EmptyBus.checked_sample)
    def test_motion_targets_and_timing_are_unchanged(self):
        class ReferencePlan(plan.Base):PLAN_VERSION=plan.PacedRetainedYawPlan.PLAN_VERSION
        old=ReferencePlan(plan.REQUEST);new=plan.PacedRetainedYawPlan(plan.REQUEST)
        a=old.plan(old.source_position());b=new.plan(new.source_position())
        self.assertEqual([p for p,q in a],[p for p,q in b]);np.testing.assert_array_equal([q for p,q in a],[q for p,q in b])
        self.assertEqual(old.SAMPLE_PERIOD,new.SAMPLE_PERIOD);self.assertEqual(new.SAMPLE_PERIOD,.075)
        self.assertEqual(old.ON_TIMEOUT,new.ON_TIMEOUT)
    def test_model_and_hardware_evidence_validate(self):
        p=plan.PacedRetainedYawPlan(plan.REQUEST);r=gate.validate_review(p,plan.PROOF)
        self.assertTrue(r['accepted_for_operator_trial']);self.assertEqual(r['fixed_hold_hardware_evidence']['hold_checks'],595)
        self.assertTrue(r['hardware_health_thresholds_unchanged']);self.assertFalse(r['automatic_goal_or_setting_resend'])
    def test_changed_passive_policy_is_rejected(self):
        p=plan.PacedRetainedYawPlan(plan.REQUEST);r=json.loads(plan.PROOF.read_bytes());r['passive_observation_policy']['period_s']=.15
        native_read=gate.base.read
        def read(path):return r if path==plan.PROOF else native_read(path)
        with patch.object(gate.base,'read',side_effect=read):
            with self.assertRaises(RuntimeError):gate.validate_review(p,plan.PROOF)
    def test_default_entry_opens_no_devices(self):
        import cv2,dynamixel_sdk
        with patch.object(sys,'argv',['leap_paced_retained_yaw_20261029.py']),patch.object(cv2,'VideoCapture',side_effect=AssertionError('Camera opened')),patch.object(dynamixel_sdk,'PortHandler',side_effect=AssertionError('Motor opened')),contextlib.redirect_stdout(io.StringIO()):self.assertEqual(entry.main(),0)
if __name__=='__main__':unittest.main(verbosity=2)
