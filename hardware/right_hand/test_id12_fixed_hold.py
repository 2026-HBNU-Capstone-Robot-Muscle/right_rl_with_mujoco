import unittest
from unittest.mock import patch
import numpy as np
import id12_fixed_hold as hold
import leap_sac_path_empty as runner
from test_leap_sac_path_empty import Clock
from test_leap_sac_start_empty import FakeBus

class HoldTest(unittest.TestCase):
    def test_prefix_before_failed_target(self):
        _,prefix=runner.load_hold_diagnostic();_,full=runner.load_diagnostic_path()
        self.assertEqual(len(prefix),40)
        np.testing.assert_array_equal(prefix,full[:40])
        self.assertEqual(prefix[-1,11],3290)
        self.assertLess(prefix[:,11].max(),3350)

    def test_failed_audit_rejected(self):
        with patch.object(runner,'HOLD_AUDIT_SHA','invalid'):
            with self.assertRaisesRegex(ValueError,'audit changed'):runner.load_hold_diagnostic()

    def test_fixed_goal_never_rewritten_and_stops_after_one_second(self):
        clock=Clock();bus=FakeBus();report={};goal=np.full(16,3290);seen=[]
        def sample(b,c,r,phase):
            seen.append(c.copy())
            actual=goal.copy();actual[11]-=max(10,80-len(seen)*4)
            return dict(position=actual,current=np.full(16,100)),np.abs(actual-goal)*runner.checked.TICK
        with patch.object(hold.checked,'sample',side_effect=sample),patch.object(hold.checked,'check_cancel'):
            hold.observe(bus,goal,report,now=clock.now,sleep=clock.sleep)
        self.assertEqual(bus.writes,[])
        self.assertTrue(report['fixed_hold']['completed'])
        self.assertGreaterEqual(clock.t,1.)
        self.assertLessEqual(clock.t,1.051)
        for command in seen:np.testing.assert_array_equal(command,goal)
        self.assertLess(report['fixed_hold']['tracking_end_median_rad'][11],report['fixed_hold']['tracking_start_rad'][11])
        self.assertFalse(report.get('path_completed',False))

    def test_stationary_joint_is_recorded_without_extending_hold(self):
        clock=Clock();bus=FakeBus();report={};goal=np.full(16,3290)
        state=dict(position=goal-70,current=np.full(16,140))
        with patch.object(hold.checked,'sample',return_value=(state,np.full(16,.107))),patch.object(hold.checked,'check_cancel'):
            hold.observe(bus,goal,report,now=clock.now,sleep=clock.sleep)
        self.assertEqual(bus.writes,[])
        self.assertEqual(report['fixed_hold']['actual_start'],report['fixed_hold']['actual_end_median'])
        self.assertLessEqual(clock.t,1.051)

    def test_cancel_has_no_write(self):
        bus=FakeBus();clock=Clock()
        with patch.object(hold.checked,'check_cancel',side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):hold.observe(bus,np.full(16,3290),{},now=clock.now,sleep=clock.sleep)
        self.assertEqual(bus.writes,[])

    def test_hold_read_failure_turns_off_and_restores(self):
        bus=FakeBus();initial=bus.snapshot();report={};clock=Clock()
        with patch.object(runner.checked,'check_cancel'),patch.object(runner.checked.time,'sleep'),\
             patch.object(runner.checked,'move',return_value=runner.target_pose()),\
             patch.object(runner.checked,'sample',side_effect=RuntimeError('hold read fault')):
            with self.assertRaisesRegex(RuntimeError,'hold read fault'):
                runner.checked.run(bus,initial,runner.plan(initial['position'],runner.target_pose()),report,empty_check=True,
                    prepared_callback=lambda b,c,r:hold.observe(b,c,r,now=clock.now,sleep=clock.sleep))
        self.assertTrue(bus.off_called);self.assertTrue(report['all_torque_off_verified']);self.assertTrue(report['settings_restored'])
        for key in ('torque','current_cap','watchdog','p_gain','i_gain','d_gain'):
            np.testing.assert_array_equal(bus.state[key],initial[key])

if __name__=='__main__':unittest.main()
