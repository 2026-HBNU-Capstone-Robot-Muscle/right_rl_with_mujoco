import unittest
from unittest.mock import patch
import numpy as np
import leap_sac_path_empty as runner
from test_leap_sac_start_empty import FakeBus

class Clock:
    def __init__(self):self.t=0.
    def now(self):return self.t
    def sleep(self,seconds):self.t+=seconds

class EmptyPathTest(unittest.TestCase):
    def test_half_speed_preserves_every_original_endpoint_and_bounds(self):
        _,original=runner.load_path();_,slow=runner.load_diagnostic_path()
        self.assertEqual(len(slow),238)
        np.testing.assert_array_equal(slow[1::2],original)
        before=np.vstack([runner.target_pose(),original[:-1]])
        self.assertTrue(np.all(slow[::2]>=np.minimum(before,original)))
        self.assertTrue(np.all(slow[::2]<=np.maximum(before,original)))
        self.assertLessEqual(np.max(np.abs(np.diff(np.vstack([runner.target_pose(),slow]),axis=0))),35)
        self.assertTrue(np.all(slow[:,15]<=3238))

    def test_changed_half_speed_audit_refused(self):
        with patch.object(runner,'AUDIT_SHA','invalid'):
            with self.assertRaisesRegex(ValueError,'audit changed'):runner.load_diagnostic_path()

    def test_same_packet_adds_pwm_velocity_and_retains_health_fields(self):
        bus=object.__new__(runner.DiagnosticBus)
        with patch.object(bus,'block',return_value={}) as block:
            bus.snapshot()
        address,length,fields=block.call_args.args
        self.assertEqual((address,length),(64,83))
        for field in [('present_pwm',124,2),('present_velocity',128,4),('goal_pwm',100,2),
                      ('torque',64,1),('error',70,1),('watchdog',98,1),('voltage',144,2),
                      ('temperature',146,1),('goal',116,4),('current_cap',102,2),('p_gain',84,2)]:
            self.assertIn(field,fields)

    def test_negative_active_telemetry_saved_before_health_fault(self):
        bus=FakeBus();state=bus.snapshot();state.update(present_pwm=np.full(16,65526),
            present_velocity=np.full(16,4294967294,dtype=np.int64),goal_pwm=np.full(16,885))
        report={'enabled_at':0.}
        with patch.object(bus,'snapshot',return_value=state),patch.object(runner.checked,'check_cancel'):
            with self.assertRaises(RuntimeError):runner.checked.sample(bus,state['goal'],report,'test_off')
        row=report['samples'][-1]
        self.assertEqual(row['present_pwm_raw'],[-10]*16)
        self.assertEqual(row['present_velocity_raw'],[-2]*16)
        self.assertEqual(row['goal_pwm_raw'],[885]*16)

    def test_large_retimed_jump_refused_before_write(self):
        bus=FakeBus();start=runner.target_pose();clock=Clock()
        with patch.object(runner.checked,'sample',return_value=({},np.zeros(16))),patch.object(runner.checked,'check_cancel'):
            with self.assertRaisesRegex(RuntimeError,'Unexpected path jump'):
                runner.replay(bus,start,{},np.array([start+36]),now=clock.now,sleep=clock.sleep)
        self.assertEqual(bus.writes,[])

    def test_pinned_path_and_mapping(self):
        doc,goals=runner.load_path()
        self.assertEqual(len(goals),119)
        np.testing.assert_array_equal(doc['start_ticks'],runner.target_pose())
        import json
        row=next(r for r in json.loads((runner.ROOT/'sac_reachable_trace_search_result.json').read_text())['results']
                 if r['compensation_rad']==.07 and r['time_scale']==1.)
        from leap_sac_start_empty import SIM_TO_ID
        restored=goals[:89,SIM_TO_ID-1]*runner.checked.TICK-np.pi
        expected=np.array(row['goals_sim_order'][1:]);expected[:,11]=np.minimum(expected[:,11],3238*runner.checked.TICK-np.pi)
        np.testing.assert_allclose(restored,expected,atol=runner.checked.TICK/2)

    def test_cap_and_thumb_first_preparation(self):
        doc,goals=runner.load_path();self.assertTrue(np.all(goals[:,15]<=3238))
        poses=runner.plan(np.full(16,2048),runner.target_pose())
        self.assertEqual(len(poses),4)
        np.testing.assert_array_equal(poses[1][1][:4],runner.target_pose()[:4])
        np.testing.assert_array_equal(poses[1][1][4:],poses[0][1][4:])
        self.assertEqual(poses[2][1][15],3238)

    def test_tampered_manifest_refused(self):
        with patch.object(runner,'PATH_SHA','invalid'):
            with self.assertRaisesRegex(ValueError,'changed'):runner.load_path()

    def test_no_catchup_burst_after_slow_read(self):
        clock=Clock();bus=FakeBus();start=runner.target_pose();goals=np.array([start+1,start+2,start+3]);times=[]
        original=bus.write
        def write(*args,**kwargs):times.append(clock.now());return original(*args,**kwargs)
        def sample(*args):clock.sleep(.08);return {},np.zeros(16)
        bus.write=write;report={}
        with patch.object(runner.checked,'check_cancel'),patch.object(runner.checked,'sample',side_effect=sample):
            runner.replay(bus,start,report,goals,now=clock.now,sleep=clock.sleep)
        self.assertTrue(np.all(np.diff(times)>=.099))
        self.assertEqual(report['path_commands_sent'],3)

    def test_failed_read_stops_without_next_goal_or_retransmit(self):
        clock=Clock();bus=FakeBus();start=runner.target_pose();report={}
        with patch.object(runner.checked,'check_cancel'),patch.object(runner.checked,'sample',side_effect=[({},np.zeros(16)),RuntimeError('bad read')]):
            with self.assertRaisesRegex(RuntimeError,'bad read'):
                runner.replay(bus,start,report,np.array([start+1,start+2]),now=clock.now,sleep=clock.sleep)
        self.assertEqual(len(bus.writes),1);self.assertEqual(report['path_write_attempts'],1)
        self.assertFalse(report.get('path_completed',False))

    def test_path_fault_uses_shared_off_and_restore(self):
        bus=FakeBus();initial=bus.snapshot();report={};clock=Clock()
        def fault_sample(*args):raise RuntimeError('path fault')
        with patch.object(runner.checked,'check_cancel'),patch.object(runner.checked.time,'sleep'),\
             patch.object(runner.checked,'move',return_value=runner.target_pose()),patch.object(runner.checked,'sample',side_effect=fault_sample):
            with self.assertRaisesRegex(RuntimeError,'path fault'):
                runner.checked.run(bus,initial,runner.plan(initial['position'],runner.target_pose()),report,empty_check=True,
                                   prepared_callback=lambda b,c,r:runner.replay(b,c,r,np.array([c,c]),now=clock.now,sleep=clock.sleep))
        self.assertTrue(bus.off_called);self.assertTrue(report['settings_restored']);self.assertTrue(report['all_torque_off_verified'])
        for key in ('torque','current_cap','p_gain','i_gain','d_gain','watchdog','velocity','acceleration'):
            np.testing.assert_array_equal(bus.state[key],initial[key])

    def test_cancel_prevents_path_goal(self):
        bus=FakeBus();start=runner.target_pose();clock=Clock()
        with patch.object(runner.checked,'sample',return_value=({},np.zeros(16))),patch.object(runner.checked,'check_cancel',side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):runner.replay(bus,start,{},np.array([start]),now=clock.now,sleep=clock.sleep)
        self.assertEqual(bus.writes,[])

if __name__=='__main__':unittest.main()
