import unittest
from unittest.mock import patch
import numpy as np
import id12_backoff as probe
from id12_backoff_spec import targets
from test_leap_sac_path_empty import Clock
from test_leap_sac_start_empty import FakeBus

class BackoffTest(unittest.TestCase):
    def test_audited_loader_and_changed_audit_rejection(self):
        import leap_sac_path_empty as runner
        _,prefix=runner.load_backoff_diagnostic()
        self.assertEqual(prefix.shape,(40,16));self.assertEqual(prefix[-1,11],3290)
        with patch.object(runner,'BACKOFF_AUDIT_SHA','changed'):
            with self.assertRaisesRegex(ValueError,'Backoff audit changed'):runner.load_backoff_diagnostic()

    def test_prefix_callback_invokes_return_instead_of_extra_hold(self):
        import leap_sac_path_empty as runner
        goal=np.full(16,3290);report={};bus=FakeBus()
        with patch.object(runner,'replay') as replay,patch.object(probe,'run') as returned,\
             patch('id12_fixed_hold.observe') as fixed:
            runner.run_hold_diagnostic(bus,goal,report,np.repeat(goal[None],40,axis=0),backoff=True)
        replay.assert_called_once();returned.assert_called_once();fixed.assert_not_called()
        np.testing.assert_array_equal(returned.call_args.args[1],goal)

    def test_only_id12_decreases_inside_exercised_envelope(self):
        start=np.arange(16)+2800;start[11]=3290
        rows=targets(start)
        self.assertEqual(rows.shape,(32,16));self.assertEqual(rows[-1,11],3130)
        np.testing.assert_array_equal(np.diff(np.r_[3290,rows[:,11]]),np.full(32,-5))
        keep=[i for i in range(16) if i!=11]
        np.testing.assert_array_equal(rows[:,keep],np.repeat(start[None,keep],32,axis=0))
        for invalid in (np.full(16,3200),np.full(16,np.nan),np.full(15,3290),np.full(16,3290.1)):
            with self.assertRaises(ValueError):targets(invalid)

    def test_32_writes_two_read_only_holds_no_forward_retry(self):
        clock=Clock();bus=FakeBus();report={};goal=np.full(16,3290)
        def sample(b,c,r,phase):return {'position':c.copy(),'current':np.zeros(16,dtype=int)},np.zeros(16)
        with patch.object(probe.checked,'sample',side_effect=sample),patch.object(probe.checked,'check_cancel'):
            probe.run(bus,goal,report,now=clock.now,sleep=clock.sleep)
        self.assertEqual(report['backoff']['commands_sent'],32)
        self.assertTrue(report['approach_hold']['completed']);self.assertTrue(report['return_hold']['completed'])
        self.assertEqual(report['backoff']['actual_change_ticks'],-160)
        self.assertAlmostEqual(clock.t,3.6,places=6)
        self.assertFalse(report.get('path_completed',False))

    def test_read_failure_before_write_aborts_and_enclosing_run_restores(self):
        bus=FakeBus();initial=bus.snapshot();report={};clock=Clock()
        from id12_stiffness_profile import make_profile
        bus.checked_control_profile=make_profile()
        def sample(b,c,r,phase):
            if phase=='id12_backoff':raise RuntimeError('return read failed')
            return {'position':c.copy(),'current':np.zeros(16,dtype=int)},np.zeros(16)
        with patch.object(probe.checked,'check_cancel'),patch.object(probe.checked.time,'sleep'),\
             patch.object(probe.checked,'move',return_value=np.full(16,3290)),\
             patch.object(probe.checked,'sample',side_effect=sample):
            with self.assertRaisesRegex(RuntimeError,'return read failed'):
                probe.checked.run(bus,initial,[('hold',initial['position'])],report,empty_check=True,
                    prepared_callback=lambda b,c,r:probe.run(b,c,r,now=clock.now,sleep=clock.sleep))
        self.assertEqual(report['backoff']['write_attempts'],0)
        self.assertTrue(report['all_torque_off_verified']);self.assertTrue(report['settings_restored'])

    def test_delayed_read_exhausts_deadline_without_goal_write(self):
        bus=FakeBus();clock=Clock();report={};goal=np.full(16,3290)
        def sample(b,c,r,phase):
            clock.sleep(4.1)
            return {'position':c.copy(),'current':np.zeros(16,dtype=int)},np.zeros(16)
        with patch.object(probe,'observe'),patch.object(probe.checked,'check_cancel'),\
             patch.object(probe.checked,'sample',side_effect=sample):
            with self.assertRaisesRegex(RuntimeError,'exceeded 4s'):
                probe.run(bus,goal,report,now=clock.now,sleep=clock.sleep)
        self.assertEqual(report['backoff']['write_attempts'],0);self.assertEqual(bus.writes,[])

if __name__=='__main__':unittest.main()
