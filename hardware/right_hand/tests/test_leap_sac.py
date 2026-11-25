"""Deployment integration with real weights and simulated motor/camera I/O only."""
import unittest
from unittest.mock import patch
import numpy as np
import bounded_policy as bounded
import held_policy_shadow as entry
import hand_prepare_full as full
import leap_sac
from test_loaded_policy_trial import Tests as LoadedTests
from test_hand_prepare_full import full_fixture


class Tests(unittest.TestCase):
    def test_20s_policy_exceeds_old_anchor_bound_but_never_joint_or_step_limits(self):
        r,error,events=LoadedTests().case(model=lambda obs:np.ones(16),joint_limits=True,duration_s=20)
        self.assertIsNone(error)
        self.assertTrue(r['loaded_trial_complete'])
        goals=np.array([e['goal'] for e in events])
        self.assertGreater(np.max(abs(goals-np.array(r['goal_hw_raw']))),650)
        q=np.array([bounded.shadow.hw_to_sim(g*bounded.TICK) for g in goals])
        self.assertTrue(np.all(q>=bounded.shadow.limits.SAFE_LOWERS))
        self.assertTrue(np.all(q<=bounded.shadow.limits.SAFE_UPPERS))
        self.assertEqual(r['policy_duration_s'],20)
        self.assertIsNone(r['policy_command_radius_rad'])
        self.assertGreater(r.get('policy_unchanged_target_count',0),0)
        self.assertFalse(r['policy_pose_estimate_summary']['physical_success_verified'])

    def test_real_deployed_npz_drives_beyond_65_ticks(self):
        policy=bounded.shadow.Policy(bounded.shadow.PROJECT/'policy/policy_np.npz')
        r,error,events=LoadedTests().case(model=policy,joint_limits=True,duration_s=20)
        self.assertIsNone(error)
        self.assertGreater(np.max(abs(np.array([e['goal'] for e in events])-np.array(r['goal_hw_raw']))),65)
        self.assertTrue(all(np.isfinite(x['action']).all() for x in r['policy_trial_samples']))

    def test_cancel_faults_and_missing_camera_keep_off_cleanup(self):
        for scenario in ('cancel_before_place','cancel_policy','cancel_remove','camera_lost',
                         'overcurrent','communication','write_fail','bad_action','missing_pose'):
            with self.subTest(scenario=scenario):
                r,error,_=LoadedTests().case(scenario,joint_limits=True,duration_s=20)
                self.assertIsNotNone(error)
                self.assertFalse(r['loaded_trial_complete'])

    def test_measured_joint_limit_rejects_without_send(self):
        bus,r,clock=full_fixture()
        anchor=full.plan(bus.snapshot()['position'])[2]
        observed=anchor.copy();writes=[]
        control=bounded.Control(anchor,read=lambda *args:dict(position=observed.copy()),
            send=writes.append,report=r,now=clock.now,step_ticks=32,joint_limits=True)
        observed[0]=control.high[0]+1
        clock.sleep(.1)
        with self.assertRaisesRegex(RuntimeError,'Measured joint outside'):
            control.apply(np.ones(16),clock.now())
        self.assertEqual(writes,[])

    def test_deployment_must_be_explicit_before_any_device_use(self):
        with patch.object(entry,'Camera',side_effect=AssertionError('No device use')):
            with self.assertRaises(ValueError):entry.main(sac_deployment_mode=True)
        bus,r,clock=full_fixture()
        r['loaded_policy_mode']=True
        with self.assertRaisesRegex(ValueError,'explicit SAC'):
            full.run_trial(bus,bus.snapshot(),r,policy_joint_limits=True,active_check=lambda c:None)
        self.assertEqual(bus.writes,[])
        with patch.object(leap_sac,'uncleared_off_failure',return_value=None):
            with patch.object(leap_sac,'run_main',return_value=0) as main:
                self.assertEqual(leap_sac.main(),0)
        self.assertEqual(main.call_args.kwargs['result_stem'],'leap_sac')
        self.assertTrue(main.call_args.kwargs['sac_deployment_mode'])
