"""Fault-injection checks; no real serial port or motor is used."""
import contextlib
import copy
import io
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import leap_measured_grip_hold_20261117 as pose
import grip_transport_recovery_20261125 as repair
from test_measured_grip_hold_20261117 import state


class Packet:
    def __init__(self, initial):
        self.state = copy.deepcopy(initial)
        self.calls = []
        self.broadcast_delivered = True
        self.silent = False
        self.fail_read_id = None
        self.pending = None

    def write1ByteTxOnly(self, port, motor, address, value):
        self.calls.append(('write_only', motor, address, value))
        if self.broadcast_delivered:
            self.state['torque'][:] = value
        return 0

    def write1ByteTxRx(self, port, motor, address, value):
        self.calls.append(('write_ack', motor, address, value))
        self.state['torque'][motor-1] = value
        return (-3001, 0) if self.silent else (0, 0)

    def readTx(self, port, motor, address, length):
        self.pending = motor, address, length
        self.calls.append(('read', motor, address, length))
        return 0

    def readRx(self, port, motor, length):
        if self.silent or motor == self.fail_read_id:
            return [], -3001, 0
        _, address, length = self.pending
        if address == 64 and length == 1:
            return [int(self.state['torque'][motor-1])], 0, 0
        data = bytearray(83)
        for name, at, size in pose.FIELDS:
            data[at-64:at-64+size] = int(self.state[name][motor-1]).to_bytes(size, 'little')
        return list(data), 0, 0


def bus_for(torque=1):
    bus = object.__new__(repair.RecoveryBus)
    initial = state(torque)
    initial['watchdog'][:] = 25 if torque else 0
    bus.packet = Packet(initial)
    bus.port = SimpleNamespace(is_using=False, getBytesAvailable=lambda:0,
                              setPacketTimeoutMillis=lambda value:None)
    bus.report = dict(enabled_at=time.monotonic())
    bus.sdk = SimpleNamespace(COMM_SUCCESS=0)
    bus.cancel_callback = lambda:None
    bus.read_only = False
    bus.controlled = True
    bus._using_unicast = bus._shutdown_started = bus._off_verified = False
    bus._read_recovery_times = []
    bus.plan_module = SimpleNamespace(NEW_PAIR_GRASP_MODE=True)
    bus.checked_control_profile = dict(caps=initial['current_cap'],
        gains={k:initial[k] for k in ('p_gain', 'i_gain', 'd_gain')})
    bus.last_goal_command = initial['goal'].copy()
    bus.previous_goal_command = None
    bus.goal_readback_confirmation = False
    return bus


class RepairTests(unittest.TestCase):
    def test_driver_observer_records_existing_error_and_restores_function(self):
        import ctypes as C
        from serial_error_observer_20261125 import SerialErrorObserver

        class Stat(C.Structure):
            _fields_ = [('cbInQue', C.c_uint32), ('cbOutQue', C.c_uint32)]

        calls = []

        def original(handle, flags, stat):
            calls.append(handle)
            C.cast(flags, C.POINTER(C.c_uint32)).contents.value = 8
            return 1

        native = SimpleNamespace(ClearCommError=original, DWORD=C.c_uint32, COMSTAT=Stat, CE_FRAME=8)
        port = SimpleNamespace(_port_handle=123, baudrate=1000000, timeout=.001)
        report = {}
        observer = SerialErrorObserver(port, report, native)
        flags, stat = C.c_uint32(), Stat(5, 0)
        self.assertEqual(native.ClearCommError(123, C.byref(flags), C.byref(stat)), 1)
        self.assertEqual(calls, [123])
        self.assertEqual(flags.value, 8)
        event = report['windows_uart_observer']['driver_error_events'][0]
        self.assertEqual((event['error_mask'], event['input_queue_bytes']), (8, 5))
        self.assertEqual(report['windows_uart_observer']['new_driver_calls'], 0)
        native.ClearCommError(999, C.byref(flags), C.byref(stat))
        self.assertEqual(len(report['windows_uart_observer']['driver_error_events']), 1)
        observer.close()
        self.assertIs(native.ClearCommError, original)

    def test_driver_observer_preserves_failure_without_decoding_pointers(self):
        from serial_error_observer_20261125 import SerialErrorObserver
        original = lambda *args:0
        native = SimpleNamespace(ClearCommError=original)
        port = SimpleNamespace(_port_handle=123, baudrate=1000000, timeout=.001)
        observer = SerialErrorObserver(port, {}, native)
        self.assertEqual(native.ClearCommError(123, None, None), 0)
        observer.close()
        self.assertIs(native.ClearCommError, original)

    def test_failed_sync_requires_two_fresh_full_unicast_states_before_return(self):
        from calibrated_two_full_health_20261028 import sample
        bus = bus_for()
        clean = bus.packet.state
        with patch('grip_health_read_20261118.block_once', return_value=(None, -3001, 0, {})) as primary, \
                patch.object(bus, '_unicast_full_health', return_value=(clean, 0, 0, {})) as fallback:
            result, _ = sample(bus, clean['goal'], bus.report, 'test')
        self.assertIs(result, clean)
        self.assertEqual(primary.call_count, 1)
        self.assertEqual(fallback.call_count, 2)
        self.assertEqual(bus.report['read_recovery_confirmation_events'][0]['clean_reads'], 2)
        self.assertFalse(any(c[0].startswith('write') for c in bus.packet.calls))

    def test_second_fresh_state_overcurrent_is_rejected(self):
        from calibrated_two_full_health_20261028 import sample
        bus = bus_for()
        clean = bus.packet.state
        bad = copy.deepcopy(clean)
        bad['current'][0] = 600
        with patch('grip_health_read_20261118.block_once', return_value=(None, -3001, 0, {})), \
                patch.object(bus, '_unicast_full_health', side_effect=[(clean, 0, 0, {}), (bad, 0, 0, {})]):
            with self.assertRaisesRegex(RuntimeError, 'current'):
                sample(bus, clean['goal'], bus.report, 'test')

    def test_full_health_device_error_never_selects_fallback(self):
        bus = bus_for()
        bus._horizontal_sample_deadline = time.perf_counter()+.2
        with patch('grip_health_read_20261118.block_once', return_value=(None, 0, 128, {})), \
                patch.object(bus, '_unicast_full_health') as fallback:
            with self.assertRaises(RuntimeError):
                bus.snapshot()
        fallback.assert_not_called()

    def test_shared_200ms_deadline_is_not_extended_for_fallback(self):
        bus = bus_for()
        bus._horizontal_sample_deadline = time.perf_counter()+.2

        def late():
            time.sleep(.21)
            return bus.packet.state, 0, 0, {}

        with patch('grip_health_read_20261118.block_once', return_value=(None, -3001, 0, {})), \
                patch.object(bus, '_unicast_full_health', side_effect=late):
            with self.assertRaisesRegex(RuntimeError, '200ms'):
                bus.snapshot()

    def test_motion_retry_limit_stays_three_per_ten_seconds(self):
        bus = bus_for()
        bus._horizontal_sample_deadline = time.perf_counter()+.2
        bus._read_recovery_times = [time.perf_counter()]*3
        with patch('grip_health_read_20261118.block_once', return_value=(None, -3001, 0, {})), \
                patch.object(bus, '_unicast_full_health') as fallback:
            with self.assertRaises(RuntimeError):
                bus.snapshot()
        fallback.assert_not_called()

    def test_missing_unicast_motor_never_yields_partial_state(self):
        bus = bus_for()
        bus.packet.fail_read_id = 7
        result, comm, error, details = bus._unicast_full_health()
        self.assertIsNone(result)
        self.assertEqual(comm, -3001)
        self.assertEqual(details['completed_ids'], list(range(1, 7)))
        self.assertEqual(details['first_incomplete_id'], 7)

    def test_all_unicast_fields_are_fresh_and_complete(self):
        bus = bus_for()
        result, comm, error, details = bus._unicast_full_health()
        self.assertEqual((comm, error), (0, 0))
        for name, _, _ in pose.FIELDS:
            np.testing.assert_array_equal(result[name], bus.packet.state[name])
        self.assertEqual(details['completed_ids'], list(range(1, 17)))

    def test_OFF_verifies_all16_even_when_motion_retry_budget_exhausted(self):
        bus = bus_for()
        bus._read_recovery_times = [time.perf_counter()]*3
        with patch.object(bus, 'snapshot', side_effect=AssertionError('Motion snapshot must not be called')):
            bus.off()
        self.assertEqual(bus.packet.calls[0], ('write_only', 254, 64, 0))
        self.assertTrue(bus.report['all_torque_off_verified'])
        self.assertEqual(len(bus._read_recovery_times), 3)
        self.assertEqual(len(bus.report['independent_shutdown_checks'][0]['readings']), 16)

    def test_lost_broadcast_falls_back_to_acknowledged_individual_OFF(self):
        bus = bus_for()
        bus.packet.broadcast_delivered = False
        bus.off()
        writes = [c for c in bus.packet.calls if c[0] == 'write_ack']
        self.assertEqual(writes, [('write_ack', i, 64, 0) for i in range(1, 17)])
        self.assertTrue(bus.report['all_torque_off_verified'])
        self.assertEqual(len(bus.report['independent_shutdown_checks']), 2)

    def test_no_replies_never_claim_OFF_or_restore_success(self):
        bus = bus_for()
        bus.packet.silent = True
        bus.report['all_torque_off_verified'] = True
        with self.assertRaisesRegex(RuntimeError, 'OFF unconfirmed'):
            bus.off()
        self.assertFalse(bus.report['all_torque_off_verified'])
        self.assertEqual(bus.report['individual_off_unconfirmed_ids'], list(range(1, 17)))
        self.assertFalse(bus.report.get('settings_restored', False))

    def test_broadcast_exception_still_attempts_individual_OFF(self):
        bus = bus_for()
        with patch.object(bus.packet, 'write1ByteTxOnly', side_effect=RuntimeError('Injected TX failure')):
            bus.off()
        self.assertTrue(bus.report['all_torque_off_verified'])
        self.assertEqual(len(bus.report['individual_OFF_transmissions']), 16)

    def test_shutdown_never_writes_goal_enable_or_early_restore(self):
        bus = bus_for()
        bus._shutdown_started = True
        for address, size, values in [(116, 4, np.full(16, 2048)), (64, 1, np.ones(16)), (98, 1, np.zeros(16))]:
            with self.assertRaises(RuntimeError):
                bus.write(address, size, values)
        self.assertEqual(bus.packet.calls, [])

    def test_read_only_cannot_send_OFF_or_configuration(self):
        bus = bus_for(0)
        bus.read_only = True
        with self.assertRaises(RuntimeError):
            bus.off()
        with self.assertRaises(RuntimeError):
            bus.write(98, 1, np.zeros(16))
        self.assertEqual(bus.packet.calls, [])

    def test_console_stop_cancels_before_reading_keyboard(self):
        import leap_vertical_direct_grip_recovery_20261125 as runner
        stop = runner.StopRequest()
        terminal = runner.terminal_for(stop)(None, {})
        stop.requested.set()
        with self.assertRaises(pose.OperatorStop):
            terminal.check()
        with self.assertRaises(pose.OperatorStop):
            terminal.wait()

    def test_default_runner_does_not_open_devices_and_restores_factory(self):
        import leap_vertical_direct_grip_recovery_20261125 as runner
        old = pose.PoseBus
        with patch.object(repair.RecoveryBus, '__init__', side_effect=AssertionError('No device construction')), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(runner.main([]), 0)
        self.assertIs(pose.PoseBus, old)


if __name__ == '__main__':
    unittest.main(verbosity=2)
