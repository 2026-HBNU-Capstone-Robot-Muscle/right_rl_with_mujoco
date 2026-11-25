"""Recorded watchdog fault and blocked UI/logging regression; no hardware."""
import contextlib
import io
import unittest
from unittest.mock import patch
import numpy as np
from test_hand_prepare_full import full_fixture
import hand_prepare_full as full


class Tests(unittest.TestCase):
    def test_input_pause_records_timing_and_watchdog_ids_then_releases(self):
        bus,report,clock=full_fixture()
        delayed=False
        def cancel():
            nonlocal delayed
            if report.get('phase')=='full_start_pose' and not delayed:
                delayed=True
                clock.sleep(.688)
                bus.state['watchdog'][:]=255
        with contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError,'Bus watchdog expired on IDs .*500ms'):
                full.run_trial(bus,bus.snapshot(),report,now=clock.now,sleep=clock.sleep,cancel=cancel)
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
        last=report['samples'][-1]
        self.assertAlmostEqual(last['cancel_duration_ms'],688)
        self.assertGreaterEqual(last['sample_interval_ms'],687)
        self.assertLess(last['snapshot_duration_ms'],200)
        self.assertEqual(last['state_all_ids']['watchdog'],[255]*16)
        self.assertEqual(len([1 for a,v in bus.writes if a==64 and np.any(v)]),1)

    def test_blocked_print_cannot_send_goal_from_old_read(self):
        bus,report,clock=full_fixture()
        at_pause=[]
        def output(*args,**kwargs):
            if args and str(args[0]).startswith('full_start_pose:'):
                at_pause.append(len([1 for a,v in bus.writes if a==116]))
                clock.sleep(.688)
        def sleep(seconds): clock.sleep(max(seconds,.06))
        with patch('builtins.print',side_effect=output):
            with self.assertRaisesRegex(RuntimeError,'state became stale before goal write'):
                full.run_trial(bus,bus.snapshot(),report,now=clock.now,sleep=sleep,cancel=lambda:None)
        self.assertTrue(at_pause)
        self.assertEqual(len([1 for a,v in bus.writes if a==116]),at_pause[-1])
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
        self.assertGreater(report['stale_preparation_advance_ms'],680)


if __name__=='__main__': unittest.main()
