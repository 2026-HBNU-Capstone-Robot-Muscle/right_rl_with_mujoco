"""Replay the actual paired corruptions and inject nonrecoverable faults."""
import copy,json,unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
import horizontal_read_recovery_20261028 as episode
import horizontal_quarter_health_20261027 as health
import sequence_flip_checked as checked
from test_horizontal_v3_health_20261027 import active
import test_horizontal_receive_resync_20261027 as transport_tests
import test_horizontal_sync_window_20261028 as window_tests

class EpisodeTests(unittest.TestCase):
    def setUp(self):
        self.clock=0.;self.command=np.full(16,2200);self.state=active(self.command)
        self.bus=SimpleNamespace(write=Mock(side_effect=AssertionError('writes forbidden')),
            plan_module=SimpleNamespace(NEW_PAIR_GRASP_MODE=True),cancel_callback=lambda:None)
        self.report={'enabled_at':0.,'read_failures':[dict(recovered=True)],'samples':[]}
        self.reads=0
    def run_confirmation(self,interruptions=(),durations=(),fault=None):
        def read():
            i=self.reads;self.reads+=1;self.clock+=durations[i] if i<len(durations) else .016
            if i in interruptions:self.report['read_failures'].append(dict(recovered=True))
            s=copy.deepcopy(self.state)
            if fault and i==fault[0]:s[fault[1]][0]=fault[2]
            self.report['samples'].append({'goal':self.command.tolist()})
            return s,np.zeros(16)
        return episode.confirm_recovered_read(self.bus,self.command,self.state,np.zeros(16),self.report,'empty_open',
            began=0.,read_sample=read,validate=lambda s:checked.health(s,self.command),now=lambda:self.clock)
    def test_normal_recovery_requires_two_additional_clean_snapshots(self):
        self.run_confirmation();self.assertEqual(self.reads,2);self.bus.write.assert_not_called()
    def test_recorded_second_corruption_resets_instead_of_stopping(self):
        source=health.checked.ROOT/'horizontal_quarter_trial_20261028_020027_500658/result.json'
        recorded=json.loads(source.read_bytes())
        cluster=recorded['read_failures'][-2:]
        self.assertEqual([f['first_incomplete_id'] for f in cluster],[7,15])
        self.clock=cluster[0]['total_elapsed_ms']/1000
        self.run_confirmation(interruptions=(0,),durations=(cluster[1]['total_elapsed_ms']/1000,.016,.016))
        event=self.report['read_recovery_confirmation_events'][-1]
        self.assertEqual(event['status'],'confirmed');self.assertEqual(event['clean_count_resets'],1)
        self.assertEqual(event['clean_reads'],2);self.assertEqual(self.reads,3)
        self.assertLess(self.clock,.2);self.bus.write.assert_not_called()
    def test_clean_then_interruption_requires_two_new_clean_reads(self):
        self.run_confirmation(interruptions=(1,));self.assertEqual(self.reads,4)
        self.assertEqual(self.report['read_recovery_confirmation_events'][-1]['consecutive_clean_sample_indices'],[2,3])
    def test_repeated_interruptions_hit_five_read_limit(self):
        with self.assertRaisesRegex(RuntimeError,'read limit'):self.run_confirmation(interruptions=range(5))
        self.assertEqual(self.reads,5);self.bus.write.assert_not_called()
    def test_two_clean_reads_after_deadline_are_rejected(self):
        with self.assertRaisesRegex(RuntimeError,'200ms'):self.run_confirmation(durations=(.11,.11))
        self.bus.write.assert_not_called()
    def test_recovered_snapshot_never_bypasses_health_or_exact_goal(self):
        for key,value in [('voltage',35),('current',551),('temperature',46),('watchdog',255),('torque',0),('position',2320),('goal',2199)]:
            with self.subTest(key=key):
                self.setUp()
                with self.assertRaises(RuntimeError):self.run_confirmation(interruptions=(0,),fault=(0,key,value))
                self.assertEqual(self.reads,1);self.bus.write.assert_not_called()
    def test_cancellation_propagates(self):
        self.bus.cancel_callback=Mock(side_effect=KeyboardInterrupt)
        with self.assertRaises(KeyboardInterrupt):self.run_confirmation()
        self.assertEqual(self.reads,0)
    def test_deadline_scope_restores_previous_even_after_error(self):
        self.bus._horizontal_sample_deadline=.1
        with self.assertRaises(ValueError):
            with episode.sample_deadline(self.bus,0.):
                self.assertEqual(self.bus._horizontal_sample_deadline,.1);raise ValueError('stop')
        self.assertEqual(self.bus._horizontal_sample_deadline,.1)
    def test_sampler_uses_new_confirmation_and_clears_deadline(self):
        count=[0]
        def reader(*args,**kwargs):
            i=count[0];count[0]+=1;self.clock+=.031 if i==0 else .047 if i==1 else .016
            if i<2:self.report['read_failures'].append(dict(recovered=True))
            return copy.deepcopy(self.state),np.zeros(16)
        with patch.object(checked,'_read_sample',side_effect=reader),patch.object(health.time,'monotonic',side_effect=lambda:self.clock):
            health.sample(self.bus,self.command,self.report,'empty_open')
        self.assertEqual(count[0],4);self.assertFalse(hasattr(self.bus,'_horizontal_sample_deadline'))
        self.assertIs(self.bus.last_reviewed_state['goal'].__class__,np.ndarray);self.bus.write.assert_not_called()
    def test_sampler_clears_deadline_when_read_fails(self):
        with patch.object(checked,'_read_sample',side_effect=RuntimeError('no reply')):
            with self.assertRaisesRegex(RuntimeError,'no reply'):health.sample(self.bus,self.command,self.report,'empty_open')
        self.assertFalse(hasattr(self.bus,'_horizontal_sample_deadline'))
    def test_proof_rejects_missing_or_changed_episode_policy(self):
        import horizontal_quarter_plan_20261027 as p
        from horizontal_quarter_gate_20261027 import validate_review
        from tempfile import TemporaryDirectory
        from pathlib import Path
        original=json.loads(p.PROOF.read_bytes())
        with TemporaryDirectory() as temporary:
            for changed in (None,dict(episode.POLICY,required_consecutive_clean_reads=1)):
                bad=copy.deepcopy(original)
                if changed is None:bad.pop('read_recovery_episode_policy',None)
                else:bad['read_recovery_episode_policy']=changed
                f=Path(temporary)/'bad.json';f.write_text(json.dumps(bad),encoding='utf-8')
                with self.assertRaisesRegex(RuntimeError,'mismatch'):validate_review(p.HorizontalQuarterPlan(p.REQUEST),f)

class DeadlineReceiveTests(unittest.TestCase):
    setUp=transport_tests.ReceiveTests.setUp
    run_read=transport_tests.ReceiveTests.run_read
    def test_outer_sample_deadline_forbids_an_extra_retry(self):
        self.bus._horizontal_sample_deadline=.1
        with self.assertRaises(RuntimeError):self.run_read([(-3002,0),(0,0)])
        self.assertEqual(self.calls,1)
    def test_outer_deadline_rejects_a_late_but_complete_snapshot(self):
        self.bus._horizontal_sample_deadline=.02
        with self.assertRaisesRegex(RuntimeError,'deadline'):self.run_read([(0,0)],duration=.021)

class DeadlineWindowTests(unittest.TestCase):
    make=window_tests.WindowTests.make
    def test_packet_window_uses_only_time_left_in_the_sample(self):
        bus,g=self.make(60);bus._horizontal_sample_deadline=10.05
        with patch('horizontal_sync_receive_window_20261028.time.monotonic',return_value=10.):
            _,comm,_,detail=__import__('horizontal_sync_receive_window_20261028').block_once(bus,64,83,[('position',132,4)])
        self.assertEqual(comm,-3001);self.assertAlmostEqual(bus.port.window_ms,50.)
        self.assertAlmostEqual(detail['receive_window_ms'],50.);bus.write.assert_not_called()
    def test_expired_episode_opens_no_new_read_transaction(self):
        bus,g=self.make(15);bus._horizontal_sample_deadline=10.
        with patch('horizontal_sync_receive_window_20261028.time.monotonic',return_value=10.):
            with self.assertRaisesRegex(RuntimeError,'deadline'):__import__('horizontal_sync_receive_window_20261028').block_once(bus,64,83,[])
        self.assertFalse(g.rx_called);self.assertTrue(g.cleared)

if __name__=='__main__':unittest.main()
