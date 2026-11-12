"""Regression tests for coarse Windows clocks and the latest200ms stop."""
import copy,math,unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
from test_horizontal_v3_health_20261027 import active
from test_horizontal_receive_resync_20261027 import Clock
import calibrated_two_full_health_20261028 as episode
import calibrated_precise_receive_20261028 as receive
import calibrated_precise_clock_20261028 as clock_policy
import sequence_flip_checked as checked
class PreciseTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.command=np.full(16,2200);self.state=active(self.command);self.calls=0
        self.report={'samples':[{'goal':self.command.tolist()}],'read_failures':[{'recovered':True}]}
        self.bus=SimpleNamespace(report=self.report,write=Mock(),plan_module=SimpleNamespace(NEW_PAIR_GRASP_MODE=True),cancel_callback=lambda:None)
    def confirm(self,durations=(.047,),interruptions=(),fault=None):
        def read():
            n=self.calls;self.calls+=1;self.clock.sleep(durations[min(n,len(durations)-1)])
            state=copy.deepcopy(self.state)
            if fault and n==fault[0]:state[fault[1]][0]=fault[2]
            if n in interruptions:self.report['read_failures'].append({'recovered':True})
            self.report['samples'].append({'goal':self.command.tolist()})
            return state,np.zeros(16)
        return episode.confirm(self.bus,self.command,self.state,np.zeros(16),self.report,'stage_3_empty_open',began=0.,read_sample=read,validate=lambda s:checked.health(s,self.command),now=self.clock.now)
    def test_logged140ms_recovered_and47ms_new_state_confirm_at187ms(self):
        self.clock.t=.140;self.confirm()
        event=self.report['read_recovery_confirmation_events'][-1]
        self.assertEqual(self.calls,1);self.assertEqual(event['clean_reads'],2);self.assertAlmostEqual(event['elapsed_ms'],187.)
        self.assertEqual(event['consecutive_clean_sample_indices'],[0,1]);self.bus.write.assert_not_called()
    def test_second_state_past200ms_is_rejected(self):
        self.clock.t=.160
        with self.assertRaisesRegex(RuntimeError,'200ms'):self.confirm()
        self.bus.write.assert_not_called()
    def test_new_corruption_resets_to_its_new_complete_state(self):
        self.clock.t=.100;self.confirm((.03,.03),interruptions=(0,))
        event=self.report['read_recovery_confirmation_events'][-1]
        self.assertEqual(self.calls,2);self.assertEqual(event['clean_count_resets'],1);self.assertEqual(event['consecutive_clean_sample_indices'],[1,2])
    def test_repeated_bad_reads_never_count_as_two_good_states(self):
        with self.assertRaisesRegex(RuntimeError,'read limit'):self.confirm((.01,),interruptions=range(5))
        self.assertEqual(self.calls,5);self.bus.write.assert_not_called()
    def test_initial_state_must_pass_original_health_and_exact_goal(self):
        for name,value in [('goal',2199),('voltage',35),('current',551),('temperature',46),('torque',0),('watchdog',255)]:
            with self.subTest(name=name):
                self.setUp();self.state[name][0]=value
                with self.assertRaises(RuntimeError):self.confirm()
                self.assertEqual(self.calls,0);self.bus.write.assert_not_called()
    def test_second_state_must_pass_original_health(self):
        for name,value in [('goal',2199),('voltage',35),('current',551),('position',2320),('temperature',46),('error',1)]:
            with self.subTest(name=name):
                self.setUp()
                with self.assertRaises(RuntimeError):self.confirm(fault=(0,name,value))
                self.bus.write.assert_not_called()
    def test_cancellation_stops_without_read_or_write(self):
        self.bus.cancel_callback=Mock(side_effect=KeyboardInterrupt)
        with self.assertRaises(KeyboardInterrupt):self.confirm()
        self.assertEqual(self.calls,0);self.bus.write.assert_not_called()
    def test_quiet_3ms_is_measured_by_precise_counter_not16ms_ticks(self):
        port=SimpleNamespace(is_using=False,getBytesAvailable=lambda:0,readPort=lambda n:bytes(n))
        self.bus.port=port
        event=receive.quiet(self.bus,now=self.clock.now,sleep=self.clock.sleep)
        self.assertAlmostEqual(event['elapsed_ms'],3.)
        self.clock.t=0.
        coarse=lambda:math.floor(self.clock.t/.015625)*.015625
        __import__('horizontal_receive_resync_20261027').quiet(self.bus,now=coarse,sleep=self.clock.sleep)
        self.assertGreaterEqual(self.clock.t,.016)
    def test_sdk_timeout_uses_QPC_even_if_wall_clock_jumps(self):
        import dynamixel_sdk as sdk
        port=sdk.PortHandler('NO_DEVICE_OPENED');bus=SimpleNamespace(port=port,report={})
        with patch.object(clock_policy.time,'perf_counter',side_effect=self.clock.now):
            clock_policy.install(bus);port.setPacketTimeoutMillis(40.)
            self.clock.t=.039;self.assertFalse(port.isPacketTimeout())
            with patch.object(clock_policy.time,'time',return_value=-1000000.):
                self.assertFalse(port.isPacketTimeout());self.clock.t=.041;self.assertTrue(port.isPacketTimeout())
        self.assertFalse(port.is_open)
    def test_sample_uses_QPC_epoch_despite_unrelated_monotonic_log_epoch(self):
        def initial(*a,**k):
            self.clock.sleep(.140);self.report['read_failures'].append({'recovered':True});return copy.deepcopy(self.state),np.zeros(16)
        def followup(*a,**k):
            self.clock.sleep(.047);self.report['samples'].append({'goal':self.command.tolist()});return copy.deepcopy(self.state),np.zeros(16)
        with patch.object(episode,'_healthy',side_effect=initial),patch.object(checked,'_read_sample',side_effect=followup),patch.object(episode.time,'perf_counter',side_effect=self.clock.now),patch.object(episode.time,'monotonic',return_value=86400.):
            episode.sample(self.bus,self.command,self.report,'empty_open')
        self.assertFalse(hasattr(self.bus,'_horizontal_sample_deadline'));self.bus.write.assert_not_called()
    def test_precise_window_and_recovery_return_only_full_new_requests(self):
        from test_single_reply40_20261028 import Fake
        import calibrated_precise_window_20261028 as precise_window
        f=Fake();request=dict(kind='sync_read',address=64,length=83)
        def read():
            state=receive.receive(f.bus,lambda:f.operation(precise_window),'health',request,now=f.now,sleep=f.sleep)
            f.bus.report['samples'].append({'goal':state['goal'].tolist()});return state,np.zeros(16)
        with patch.object(precise_window.time,'perf_counter',side_effect=f.now):
            with episode.sample_deadline(f.bus,0.):
                state,errors=read()
                state,errors=episode.confirm(f.bus,f.command,state,errors,f.bus.report,'empty_open',began=0.,read_sample=read,validate=lambda s:checked.health(s,f.command),now=f.now)
        self.assertEqual(f.n,3);self.assertTrue(np.all(state['position']==f.command+3));self.assertLess(f.t,.2)
        self.assertEqual(f.bus.report['read_failures'][0]['completed_ids'],[1,2,3,4]);f.bus.write.assert_not_called()
    def test_precise_window_respects_remaining_QPC_deadline(self):
        from test_single_reply40_20261028 import Fake
        import calibrated_precise_window_20261028 as precise_window
        f=Fake(fault=False);f.bus._horizontal_sample_deadline=.001
        with patch.object(precise_window.time,'perf_counter',side_effect=f.now):value,comm,_,_=f.operation(precise_window)
        self.assertIsNone(value);self.assertNotEqual(comm,0);self.assertLessEqual(f.windows[-1],1.);f.bus.write.assert_not_called()
if __name__=='__main__':unittest.main()
