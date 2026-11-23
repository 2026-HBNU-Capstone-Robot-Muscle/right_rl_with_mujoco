"""Offline tests: read spacing, unchanged motion retries and shutdown reads."""
import time,unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
import leap_measured_grip_hold_20261117 as pose
import vertical_direct_transport_20261124 as transport
from test_measured_grip_hold_20261117 import Clock,state

class Tests(unittest.TestCase):
    def bus(self):
        bus=object.__new__(transport.DirectPoseBus)
        bus.report={'all_torque_off_verified':False}
        bus.controlled=True;bus.cancel_callback=lambda:None
        bus._last_health_finished=None;bus._shutdown_read_only=False
        bus._read_recovery_times=[]
        return bus

    def test_motion_has_gap_after_complete_read_without_extra_reads(self):
        bus=self.bus();clock=Clock();starts=[]
        def read():starts.append(clock.now());clock.sleep(.032);return state(1)
        with patch.object(transport,'clock',clock.now),patch.object(transport,'sleep',clock.sleep),patch.object(pose.PoseBus,'snapshot',side_effect=read) as original:
            bus.snapshot();bus.snapshot()
        self.assertEqual(original.call_count,2)
        self.assertAlmostEqual(starts[1]-starts[0],.042)
        self.assertAlmostEqual(clock.t,.074)

    def test_preflight_does_not_add_pacing_or_reads(self):
        bus=self.bus();bus.controlled=False;clock=Clock()
        def read():clock.sleep(.032);return state(0)
        with patch.object(transport,'clock',clock.now),patch.object(transport,'sleep',clock.sleep),patch.object(pose.PoseBus,'snapshot',side_effect=read) as original:
            bus.snapshot();bus.snapshot()
        self.assertEqual(original.call_count,2);self.assertAlmostEqual(clock.t,.064)

    def test_cancel_during_gap_prevents_next_read(self):
        bus=self.bus();bus._last_health_finished=0;clock=Clock()
        bus.cancel_callback=Mock(side_effect=pose.OperatorStop())
        with patch.object(transport,'clock',clock.now),patch.object(transport,'sleep',clock.sleep),patch.object(pose.PoseBus,'snapshot') as original:
            with self.assertRaises(pose.OperatorStop):bus.snapshot()
        original.assert_not_called()

    def test_gap_does_not_extend_existing_sample_deadline(self):
        bus=self.bus();bus._last_health_finished=0;bus._horizontal_sample_deadline=.004;clock=Clock()
        with patch.object(transport,'clock',clock.now),patch.object(transport,'sleep',clock.sleep),patch.object(pose.PoseBus,'snapshot') as original:
            with self.assertRaisesRegex(RuntimeError,'existing sample deadline'):bus.snapshot()
        original.assert_not_called();self.assertAlmostEqual(clock.t,.004)

    def test_motion_still_stops_when_original_retry_allowance_is_used(self):
        bus=self.bus();retry_times=[time.perf_counter()]*3;bus._read_recovery_times=retry_times.copy()
        with patch('grip_health_read_20261118.block_once',return_value=(None,-3001,0,{})) as reader:
            with self.assertRaisesRegex(RuntimeError,'Existing3retries/10s'):bus.snapshot()
        self.assertEqual(reader.call_count,1)
        self.assertEqual(bus._read_recovery_times,retry_times)
        self.assertFalse(bus._shutdown_read_only)

    def shutdown_bus(self):
        bus=self.bus();bus._read_recovery_times=[time.perf_counter()]*3
        bus.write=Mock();bus.port=SimpleNamespace()
        bus.packet=SimpleNamespace(write1ByteTxRx=Mock(return_value=(0,0)))
        return bus

    def test_clean_shutdown_read_works_after_motion_retry_exhaustion(self):
        bus=self.shutdown_bus();used=bus._read_recovery_times.copy()
        with patch('grip_health_read_20261118.block_once',return_value=(state(0),0,0,{})) as reader,patch.object(pose.PoseBus,'snapshot') as motion_reader:
            bus.off()
        motion_reader.assert_not_called();self.assertEqual(reader.call_count,1)
        self.assertEqual(bus._read_recovery_times,used)
        self.assertTrue(bus.report['all_torque_off_verified'])
        self.assertTrue(bus.report['shutdown_verification_reads'][0]['all_torque_off'])
        self.assertEqual(bus.write.call_args.args[0:2],(64,1))
        self.assertTrue(np.all(bus.write.call_args.args[2]==0))

    def test_persistent_or_partial_shutdown_read_never_claims_OFF(self):
        for response in [(None,-3001,0,{}),({'torque':np.zeros(8,dtype=int)},0,0,{})]:
            bus=self.shutdown_bus()
            with patch('grip_health_read_20261118.block_once',return_value=response) as reader:
                with self.assertRaises(RuntimeError):bus.off()
            self.assertFalse(bus.report['all_torque_off_verified'])
            self.assertEqual(reader.call_count,2)
            self.assertEqual(bus.packet.write1ByteTxRx.call_count,16)
            self.assertTrue(all(call.args[0]==64 and not np.any(call.args[2]) for call in bus.write.call_args_list))

    def test_shutdown_blocks_new_goals_and_enable_but_allows_OFF(self):
        bus=self.bus();bus._shutdown_read_only=True
        with patch.object(pose.PoseBus,'write') as original:
            with self.assertRaisesRegex(RuntimeError,'prohibited'):bus.write(116,4,np.full(16,2048))
            with self.assertRaisesRegex(RuntimeError,'prohibited'):bus.write(64,1,np.ones(16))
            original.assert_not_called()
            bus.write(64,1,np.zeros(16))
            self.assertEqual(original.call_count,1)

if __name__=='__main__':unittest.main()
