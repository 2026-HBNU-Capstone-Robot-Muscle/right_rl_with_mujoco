"""Expanded loaded trial, simulated only; unchanged per-command step and OFF behavior."""
import unittest
from unittest.mock import patch
import numpy as np
import test_loaded_policy_trial as loaded_tests
from test_hand_prepare_full import full_fixture
import hand_prepare_full as full
import bounded_policy as bounded
import held_policy_shadow as entry
import leap_loaded_policy_010 as expanded


class Tests(unittest.TestCase):
    def test_saturated_policy_reaches_65_ticks_with_32_tick_steps_and_5s_limit(self):
        report,error,events=loaded_tests.Tests().case(model=lambda obs:np.ones(16),radius=65)
        self.assertIsNone(error)
        self.assertTrue(report['loaded_trial_complete'])
        anchor=np.array(report['goal_hw_raw'])
        self.assertEqual(int(np.max(abs(np.array([e['goal'] for e in events])-anchor))),65)
        self.assertEqual(report['policy_step_ticks'],32)
        self.assertEqual(report['policy_command_radius_ticks'],65)
        self.assertEqual(report['policy_measured_radius_rad'],.125)
        self.assertEqual(report['policy_duration_s'],5)

    def test_actual_npz_and_stage_cancels_faults_still_release(self):
        report,error,_=loaded_tests.Tests().case(model=bounded.shadow.Policy(bounded.shadow.PROJECT/'policy/policy_np.npz'),radius=65)
        self.assertIsNone(error)
        self.assertGreater(report['policy_actions_sent'],0)
        for scenario in ('cancel_before_place','cancel_policy','cancel_remove','camera_lost','overcurrent','communication','write_fail','bad_action'):
            with self.subTest(scenario=scenario):
                report,error,_=loaded_tests.Tests().case(scenario,radius=65)
                self.assertIsNotNone(error)
                self.assertFalse(report['loaded_trial_complete'])

    def test_measured_envelope_and_joint_limits_are_checked_before_writes(self):
        bus,report,clock=full_fixture()
        anchor=full.plan(bus.snapshot()['position'])[2]
        observed=anchor.copy()
        writes=[]
        control=bounded.Control(anchor,read=lambda *args:dict(position=observed.copy()),
            send=writes.append,report=report,now=clock.now,step_ticks=32,radius_ticks=65)
        observed[0]+=82  # 0.1258rad, outside the new measured envelope.
        clock.sleep(.1)
        with self.assertRaisesRegex(RuntimeError,'Measured joint outside'):
            control.apply(np.ones(16),clock.now())
        self.assertEqual(writes,[])

    def test_old_default_and_invalid_expansion_cannot_silently_change(self):
        report,error,_=loaded_tests.Tests().case(model=lambda obs:np.ones(16))
        self.assertIsNone(error)
        self.assertEqual(report['policy_command_radius_ticks'],32)
        self.assertEqual(report['policy_measured_radius_rad'],.075)
        bus,report,clock=full_fixture()
        with self.assertRaisesRegex(ValueError,'explicit loaded-policy'):
            full.run_trial(bus,bus.snapshot(),report,policy_radius_ticks=65,active_check=lambda c:None)
        self.assertEqual(bus.writes,[])
        with patch.object(entry,'Camera',side_effect=AssertionError('No camera should open')):
            with self.assertRaises(ValueError): entry.main(policy_radius_ticks=65)

    def test_new_launcher_has_distinct_output_and_explicit_radius(self):
        with patch.object(expanded,'run_main',return_value=0) as main:
            self.assertEqual(expanded.main(),0)
        self.assertEqual(main.call_args.kwargs['policy_radius_ticks'],65)
        self.assertEqual(main.call_args.kwargs['result_stem'],'leap_loaded_policy_010')
        self.assertTrue(main.call_args.kwargs['loaded_policy_mode'])


if __name__=='__main__': unittest.main()
