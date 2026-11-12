"""Fault injection for full-health read bursts, deadlines and cancellation."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
from horizontal_receive_resync_20261027 import receive,quiet
class Clock:
    def __init__(self):self.t=0.
    def now(self):return self.t
    def sleep(self,s):self.t+=s
class ReceiveTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.port=SimpleNamespace(is_using=False,getBytesAvailable=lambda:0,readPort=lambda n:bytes(n))
        self.bus=SimpleNamespace(port=self.port,report={},sdk=SimpleNamespace(COMM_SUCCESS=0),_read_recovery_times=[],write=Mock())
        self.request=dict(kind='sync_read',address=64,length=83);self.calls=0
    def run_read(self,results,duration=.016):
        def operation():
            self.clock.sleep(duration);self.calls+=1
            comm,error=results.pop(0)
            return {'fresh':self.calls},comm,error,dict(completed_ids=[],first_incomplete_id=1)
        return receive(self.bus,operation,'health READ',self.request,now=self.clock.now,sleep=self.clock.sleep)
    def test_normal_read_does_not_retry(self):
        self.assertEqual(self.run_read([(0,0)])['fresh'],1);self.assertNotIn('read_failures',self.bus.report)
    def test_two_corrupt_reads_then_fresh_read_recover_inside200ms(self):
        self.assertEqual(self.run_read([(-3002,0),(-3002,0),(0,0)])['fresh'],3)
        self.assertTrue(all(v['recovered'] for v in self.bus.report['read_failures']))
        self.assertLess(self.clock.t,.1);self.bus.write.assert_not_called()
    def test_timeout_can_recover(self):self.assertEqual(self.run_read([(-3001,0),(0,0)])['fresh'],2)
    def test_persistent_corruption_stops_after_three_requests(self):
        with self.assertRaises(RuntimeError):self.run_read([(-3002,0)]*3)
        self.assertEqual(self.calls,3);self.bus.write.assert_not_called()
    def test_success_after_deadline_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError,'200ms'):self.run_read([(0,0)],duration=.201)
    def test_long_first_failure_does_not_spend_reserved_confirmation_time(self):
        with self.assertRaises(RuntimeError):self.run_read([(-3002,0),(0,0)],duration=.081)
        self.assertEqual(self.calls,1)
    def test_device_error_is_not_retried(self):
        with self.assertRaises(RuntimeError):self.run_read([(0,1),(0,0)])
        self.assertEqual(self.calls,1)
    def test_existing_ten_second_retry_rate_limit_is_retained(self):
        self.bus._read_recovery_times=[0.,0.,0.]
        with self.assertRaises(RuntimeError):self.run_read([(-3002,0),(0,0)])
        self.assertEqual(self.calls,1)
    def test_pending_bytes_are_drained_before_quiet_interval(self):
        pending=iter([3,0,2,0,0,0,0,0]);self.port.getBytesAvailable=lambda:next(pending,0)
        result=quiet(self.bus,now=self.clock.now,sleep=self.clock.sleep)
        self.assertEqual(result['discarded_bytes'],5);self.assertGreaterEqual(self.clock.t,.005)
    def test_continuous_input_is_not_treated_as_recovered(self):
        self.port.getBytesAvailable=lambda:1
        with self.assertRaisesRegex(RuntimeError,'quiet'):self.run_read([(-3002,0),(0,0)])
        self.assertEqual(self.calls,1)
    def test_cancellation_does_not_send_another_request(self):
        self.bus.cancel_callback=Mock(side_effect=KeyboardInterrupt)
        with self.assertRaises(KeyboardInterrupt):self.run_read([(0,0)])
        self.assertEqual(self.calls,0)
    def test_other_registers_are_outside_the_scope(self):
        self.request['address']=98
        with self.assertRaises(ValueError):self.run_read([(0,0)])
        self.assertEqual(self.calls,0)
    def test_only_current_bus_full_health_reads_use_the_new_receiver(self):
        import leap_horizontal_quarter_20261027 as live
        bus=live.HorizontalQuarterBus.__new__(live.HorizontalQuarterBus)
        operation=lambda:None
        with patch.object(live,'receive_resynchronized',return_value='new') as new,patch.object(live.NewPairBus,'_receive',return_value='old') as old:
            self.assertEqual(bus._receive(operation,'full',self.request),'new')
            self.assertEqual(bus._receive(operation,'watchdog',dict(kind='sync_read',address=98,length=1)),'old')
            self.assertEqual(new.call_count,1);self.assertEqual(old.call_count,1)
    def test_cleanup_failure_keeps_original_motion_error(self):
        import leap_horizontal_quarter_20261027 as live
        bus=live.HorizontalQuarterBus.__new__(live.HorizontalQuarterBus)
        bus.report=dict(all_torque_off_verified=True,primary_motion_error='full health read failed')
        with patch.object(live.NewPairBus,'write',side_effect=RuntimeError('cleanup read98 failed')):
            with self.assertRaisesRegex(RuntimeError,'full health read failed.*restoration incomplete'):
                bus.write(98,1,[0]*16)
        self.assertEqual(bus.report['post_off_restore_error'],'cleanup read98 failed')
if __name__=='__main__':unittest.main()
