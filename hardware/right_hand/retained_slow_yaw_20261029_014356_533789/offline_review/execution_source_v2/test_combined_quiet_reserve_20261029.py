"""Recorded failures, delayed serial tails, scheduling and unchanged200ms limit."""
import unittest
from types import SimpleNamespace
from combined_quiet_reserve_20261029 import receive,POLICY
from calibrated_precise_receive_20261028 import receive as old_receive

REQUEST=dict(kind='sync_read',address=64,length=83)
class Clock:
    def __init__(self):self.t=0.
    def now(self):return self.t
    def sleep(self,seconds):self.t+=seconds
class Port:
    is_using=False
    def __init__(self,clock,busy_until=0.):self.clock=clock;self.busy_until=busy_until
    def getBytesAvailable(self):return int(self.clock.t<self.busy_until)
    def readPort(self,count):return b'x'*count
def setup(sequence,*,deadline=.2,busy_until=0.,previous=()):
    c=Clock();bus=SimpleNamespace(sdk=SimpleNamespace(COMM_SUCCESS=0),port=Port(c,busy_until),
        report={},_read_recovery_times=list(previous),_horizontal_sample_deadline=deadline)
    calls=[]
    def operation():
        i=len(calls);calls.append(i)
        duration,comm,error=sequence[i];c.sleep(duration)
        return ({'fresh_full_snapshot':i} if comm==0 else None),comm,error,{'first_incomplete_id':12}
    return c,bus,operation,calls

class Checks(unittest.TestCase):
    def test_recorded_failure_was_rejected_by_old_reservation(self):
        c,b,o,calls=setup([(.0806942,-3002,0),(.031,0,0)])
        with self.assertRaises(RuntimeError):old_receive(b,o,'READ',REQUEST,now=c.now,sleep=c.sleep)
        self.assertEqual(len(calls),1)
    def test_recorded_failure_can_recover_without_budget_extension(self):
        c,b,o,calls=setup([(.0806942,-3002,0),(.031,0,0)])
        value=receive(b,o,'READ',REQUEST,now=c.now,sleep=c.sleep)
        self.assertEqual(value,{'fresh_full_snapshot':1});self.assertEqual(len(calls),2)
        # The unchanged health sampler still requires a second complete READ.
        c.sleep(.031);self.assertLess(c.now(),.2)
        self.assertTrue(b.report['read_failures'][0]['recovered'])
        self.assertGreater(b.report['read_failures'][0]['remaining_budget_after_quiet_ms'],100)
    def test_serial_tail_longer_than20ms_can_fit_same200ms(self):
        c,b,o,calls=setup([(.0792898,-3002,0),(.031,0,0)],busy_until=.111)
        receive(b,o,'READ',REQUEST,now=c.now,sleep=c.sleep)
        c.sleep(.031);self.assertLess(c.now(),.2)
        self.assertGreater(b.report['read_failures'][0]['quiet_resynchronization']['elapsed_ms'],20)
    def test_shared_shorter_deadline_preserved(self):
        c,b,o,calls=setup([(.0806942,-3002,0),(.08,0,0)],deadline=.15)
        with self.assertRaises(RuntimeError):receive(b,o,'READ',REQUEST,now=c.now,sleep=c.sleep)
        self.assertEqual(len(calls),2)
    def test_three_retries_per10s_rate_limit_preserved(self):
        c,b,o,calls=setup([(.04,-3002,0)],previous=[-.1,-.2,-.3])
        with self.assertRaises(RuntimeError):receive(b,o,'READ',REQUEST,now=c.now,sleep=c.sleep)
        self.assertEqual(len(calls),1)
    def test_device_error_never_retried(self):
        c,b,o,calls=setup([(.03,0,128)])
        with self.assertRaises(RuntimeError):receive(b,o,'READ',REQUEST,now=c.now,sleep=c.sleep)
        self.assertEqual(len(calls),1)
    def test_unrecoverable_transport_error_not_retried(self):
        c,b,o,calls=setup([(.03,-1001,0)])
        with self.assertRaises(RuntimeError):receive(b,o,'READ',REQUEST,now=c.now,sleep=c.sleep)
        self.assertEqual(len(calls),1)
    def test_no_data_accepted_after_deadline(self):
        c,b,o,calls=setup([(.0806942,-3002,0),(.125,0,0)])
        with self.assertRaises(RuntimeError):receive(b,o,'READ',REQUEST,now=c.now,sleep=c.sleep)
    def test_normal_complete_read_unchanged(self):
        c,b,o,calls=setup([(.031,0,0)])
        self.assertEqual(receive(b,o,'READ',REQUEST,now=c.now,sleep=c.sleep),{'fresh_full_snapshot':0})
        self.assertFalse(b.report.get('read_failures'))
    def test_other_addresses_rejected(self):
        c,b,o,calls=setup([(.031,0,0)])
        with self.assertRaises(ValueError):receive(b,o,'READ',dict(kind='sync_read',address=116,length=4),now=c.now,sleep=c.sleep)
        self.assertFalse(calls)
    def test_continuous_noise_stops_without_new_read(self):
        c,b,o,calls=setup([(.04,-3002,0)],busy_until=.2)
        with self.assertRaises(RuntimeError):receive(b,o,'READ',REQUEST,now=c.now,sleep=c.sleep)
        self.assertEqual(len(calls),1)
    def test_policy_still_has_original_health_guarantees(self):
        self.assertEqual(POLICY['total_read_budget_s'],.2);self.assertIsNone(POLICY['predicted_retry_reservation_s'])
        self.assertEqual(POLICY['consecutive_healthy_snapshots_after_recovery'],2)
        self.assertEqual(POLICY['goal_or_setting_write_retries'],0)
    def test_second_healthy_snapshot_must_also_fit_the_original_deadline(self):
        import numpy as np
        from calibrated_two_full_health_20261028 import confirm
        c,b,o,calls=setup([(.0792898,-3002,0),(.065,0,0)],busy_until=.112)
        receive(b,o,'READ',REQUEST,now=c.now,sleep=c.sleep)
        state={'goal':np.zeros(16,dtype=int)};report={'samples':[{}]}
        def next_read():c.sleep(.04);report['samples'].append({});return state,np.zeros(16)
        with self.assertRaises(RuntimeError):confirm(b,state['goal'],state,np.zeros(16),report,'regression',
            began=0.,read_sample=next_read,validate=lambda s:None,now=c.now)
        self.assertEqual(report['read_recovery_confirmation_events'][0]['status'],'stopped')
    def test_sleep_scheduling_delay_uses_shared_deadline(self):
        c,b,o,calls=setup([(.0792898,-3002,0),(.031,0,0)])
        def delayed_sleep(seconds):c.sleep(.025)
        receive(b,o,'READ',REQUEST,now=c.now,sleep=delayed_sleep)
        c.sleep(.031);self.assertLess(c.now(),.2)
        self.assertTrue(b.report['read_failures'][0]['recovered'])
    def test_quiet_timeout_is_logged_before_stop(self):
        c,b,o,calls=setup([(.0792898,-3002,0)],busy_until=.3)
        with self.assertRaises(RuntimeError):receive(b,o,'READ',REQUEST,now=c.now,sleep=c.sleep)
        trace=b.report['read_failures'][0]['quiet_resynchronization']
        self.assertEqual(trace['status'],'stopped');self.assertTrue(trace['observations'])
    def test_new_transport_requires_two_complete_native_healthy_snapshots(self):
        from unittest.mock import patch
        import numpy as np
        from test_single_reply40_20261028 import Fake
        import calibrated_precise_window_20261028 as window
        import calibrated_two_full_health_20261028 as episode
        import sequence_flip_checked as checked
        f=Fake();request=dict(kind='sync_read',address=64,length=83)
        def fresh():
            state=receive(f.bus,lambda:f.operation(window),'health',request,now=f.now,sleep=f.sleep)
            f.bus.report['samples'].append({'goal':state['goal'].tolist()});return state,np.zeros(16)
        with patch.object(window.time,'perf_counter',side_effect=f.now):
            with episode.sample_deadline(f.bus,0.):
                state,errors=fresh()
                state,errors=episode.confirm(f.bus,f.command,state,errors,f.bus.report,'empty_open',began=0.,read_sample=fresh,
                    validate=lambda s:checked.health(s,f.command),now=f.now)
        self.assertEqual(f.n,3);self.assertTrue(np.all(state['position']==f.command+3))
        self.assertLess(f.t,.2);self.assertEqual(f.bus.report['read_recovery_confirmation_events'][-1]['clean_reads'],2)
        f.bus.write.assert_not_called()

if __name__=='__main__':unittest.main(verbosity=2)
