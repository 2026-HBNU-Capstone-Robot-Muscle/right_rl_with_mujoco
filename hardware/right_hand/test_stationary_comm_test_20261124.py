"""Offline tests of the user-started stationary transmission diagnostic."""
import contextlib,copy,io,unittest
import numpy as np
from test_measured_grip_hold_20261117 import FakeBus,Clock,state
import leap_measured_grip_hold_20261117 as pose
import stationary_comm_test_20261124 as diagnostic

class Tests(unittest.TestCase):
    def run_bus(self,bus,initial):
        clock=Clock();bus.report.update(status='started',holding=False)
        with contextlib.redirect_stdout(io.StringIO()):
            diagnostic.run(bus,initial,initial['position'].copy(),bus.report,lambda:None,
                now=clock.now,sleep=clock.sleep)
        return clock

    def test_current_goal_only_15s_and_original_settings_restored(self):
        initial=state(0);bus=FakeBus(initial);clock=self.run_bus(bus,initial)
        goals=[v for a,v in bus.writes if a==116]
        self.assertGreater(len(goals),100)
        self.assertTrue(all(np.array_equal(q,initial['position']) for q in goals))
        self.assertFalse(bus.report['rotation_included'])
        self.assertFalse(bus.report['fixed_G_transition_included'])
        self.assertEqual(bus.report['status'],'stationary_15second_test_complete')
        self.assertGreaterEqual(clock.t,15);self.assertLess(clock.t,15.2)
        self.assertTrue(bus.report['all_torque_off_verified']);self.assertTrue(bus.report['settings_restored'])

    def test_original_health_fault_stops_and_restores(self):
        initial=state(0);bus=FakeBus(initial);bus.fault_after=25
        with self.assertRaisesRegex(RuntimeError,'measured current'):self.run_bus(bus,initial)
        self.assertTrue(bus.report['all_torque_off_verified']);self.assertTrue(bus.report['settings_restored'])
        self.assertTrue(all(np.array_equal(v,initial['position']) for a,v in bus.writes if a==116))

    def test_accepted_one_tick_preload_correction_remains_bounded(self):
        class Shift(FakeBus):
            shifted=False
            def write(self,address,size,values):
                super().write(address,size,values)
                if address==102 and not self.shifted:self.state['position'][8]+=1;self.shifted=True
        initial=state(0);bus=Shift(initial);self.run_bus(bus,initial)
        goals=np.array([v for a,v in bus.writes if a==116])
        self.assertLessEqual(np.max(abs(np.diff(goals,axis=0))),1)
        self.assertTrue(all(np.array_equal(q,initial['position']) for q in goals[1:]))
        self.assertTrue(bus.report['all_torque_off_verified'])

    def test_outward_motion_from_outside_range_is_still_blocked_before_enable(self):
        class Shift(FakeBus):
            shifted=False
            def write(self,address,size,values):
                super().write(address,size,values)
                if address==102 and not self.shifted:self.state['position'][15]+=1;self.shifted=True
        initial=copy.deepcopy(state(0));low=pose.bounds()[2]
        initial['position'][15]=int(np.ceil(low[15]))-20;initial['goal']=initial['position'].copy()
        bus=Shift(initial)
        with self.assertRaisesRegex(RuntimeError,'range/return envelope'):self.run_bus(bus,initial)
        self.assertFalse(any(a==64 and np.any(v) for a,v in bus.writes))
        self.assertTrue(bus.report['all_torque_off_verified'])

    def test_initial_ON_is_rejected_without_writes(self):
        initial=state(1);bus=FakeBus(initial)
        with self.assertRaisesRegex(RuntimeError,'initial torque OFF'):self.run_bus(bus,initial)
        self.assertEqual(bus.writes,[])

    def test_unconfirmed_OFF_does_not_report_completed_test(self):
        class FailedOFF(FakeBus):
            def off(self):raise RuntimeError('Synthetic OFF communication failure')
        initial=state(0);bus=FailedOFF(initial);self.run_bus(bus,initial)
        self.assertEqual(bus.report['status'],'stationary_cleanup_failed')
        self.assertIn('OFF communication failure',bus.report['cleanup_error'])
        self.assertFalse(bus.report.get('all_torque_off_verified',False))

if __name__=='__main__':unittest.main()
