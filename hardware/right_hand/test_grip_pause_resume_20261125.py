"""Meaningful fault-injection checks; no serial port or real motor is opened."""
import contextlib
import copy
import io
import time
import unittest
from unittest.mock import patch

import numpy as np

import grip_pause_resume_20261125 as pause
from test_grip_transport_recovery_20261125 import bus_for


def paused_bus():
    bus = bus_for()
    bus.__class__ = pause.PauseBus
    bus._motion_paused = False
    bus.report['accepted_start_goal'] = bus.last_goal_command.tolist()
    bus.report['completed_goals'] = 1247
    bus.last_reviewed_state = copy.deepcopy(bus.packet.state)
    return bus


class Clock:
    def __init__(self):
        self.at = 100.

    def __call__(self):
        return self.at


class PauseTests(unittest.TestCase):
    def resume(self, bus, sequence, *, check=None, duration=.03):
        clock = Clock()
        iterator = iter(sequence)

        def probe(method, deadline):
            clock.at += duration
            value = next(iterator)
            if callable(value):
                value = value(bus)
            if isinstance(value, BaseException):
                raise value
            return value

        with patch('combined_quiet_reserve_20261029.quiet'), \
                patch.object(bus, 'pause_probe', side_effect=probe), \
                contextlib.redirect_stdout(io.StringIO()):
            return pause.resume_health(bus, bus.last_goal_command, bus.report, 'direct_entry_support',
                pause.LinkReadFailure('Injected missing reply'), snapshot_check=check,
                now=clock, sleep=lambda value:None)

    def test_timeout_then_two_fresh_full_states_resumes_same_goal_without_writes(self):
        bus = paused_bus()
        clean = bus.packet.state
        with patch.object(bus, 'write', side_effect=AssertionError('No writes during pause')):
            result, _ = self.resume(bus, [(None, -3001, 0, {}),
                (clean, 0, 0, {}), (copy.deepcopy(clean), 0, 0, {})])
        np.testing.assert_array_equal(result['goal'], bus.last_goal_command)
        self.assertEqual(bus.report['completed_goals'], 1247)
        event = bus.report['communication_pause_events'][0]
        self.assertEqual((event['status'], event['clean_reads']), ('resumed', 2))
        self.assertEqual(event['completed_goals_before'], event['completed_goals_after'])
        self.assertEqual(bus.packet.calls, [])
        self.assertFalse(bus._motion_paused)

    def test_one_good_reply_followed_by_failure_does_not_count_as_two(self):
        bus = paused_bus()
        clean = bus.packet.state
        self.resume(bus, [(clean, 0, 0, {}), (None, -3002, 0, {}),
                          (clean, 0, 0, {}), (clean, 0, 0, {})])
        event = bus.report['communication_pause_events'][0]
        self.assertEqual(len(event['attempts']), 4)
        self.assertEqual(event['clean_reads'], 2)

    def test_permanent_silence_expires_without_changing_goal_or_enabling(self):
        bus = paused_bus()
        goal = bus.last_goal_command.copy()
        with self.assertRaisesRegex(RuntimeError, 'not recovered within1s'):
            self.resume(bus, [(None, -3001, 0, {})]*10, duration=.19)
        self.assertEqual(bus.packet.calls, [])
        np.testing.assert_array_equal(goal, bus.last_goal_command)
        self.assertFalse(bus._motion_paused)
        self.assertEqual(bus.report['communication_pause_events'][0]['status'], 'stopped')

    def test_overcurrent_watchdog_off_and_hardware_error_stop_without_second_read(self):
        for name, value in [('current', 600), ('watchdog', 255), ('torque', 0), ('error', 32),
                            ('voltage', 35), ('temperature', 50), ('current_cap', 500)]:
            with self.subTest(name=name):
                bus = paused_bus()
                bad = copy.deepcopy(bus.packet.state)
                bad[name][0] = value
                with self.assertRaises(RuntimeError):
                    self.resume(bus, [(bad, 0, 0, {})])
                self.assertEqual(len(bus.report['communication_pause_events'][0]['attempts']), 1)
                self.assertFalse(bus._motion_paused)

    def test_changed_goal_is_not_replayed_or_overwritten(self):
        bus = paused_bus()
        bad = copy.deepcopy(bus.packet.state)
        bad['goal'][0] += 1
        with self.assertRaises(pause.checked.GoalReadbackMismatch):
            self.resume(bus, [(bad, 0, 0, {})])
        self.assertEqual(bus.packet.calls, [])

    def test_pose_drift_and_current_goal_tracking_block_resume(self):
        for change in ('tracking', 'drift'):
            with self.subTest(change=change):
                bus = paused_bus()
                bad = copy.deepcopy(bus.packet.state)
                if change == 'tracking':
                    bad['position'][0] += 79
                else:
                    bus.last_reviewed_state['position'][0] -= 79
                with self.assertRaisesRegex(RuntimeError, '0.12rad'):
                    self.resume(bus, [(bad, 0, 0, {})])

    def test_held_joint_guard_and_device_packet_error_are_not_retried(self):
        bus = paused_bus()
        with self.assertRaisesRegex(RuntimeError, 'Held joint'):
            self.resume(bus, [(bus.packet.state, 0, 0, {})],
                check=lambda state:(_ for _ in ()).throw(RuntimeError('Held joint changed')))
        bus = paused_bus()
        with self.assertRaisesRegex(RuntimeError, 'device/transport'):
            self.resume(bus, [(None, 0, 128, {})])

    def test_partial_state_is_never_accepted(self):
        bus = paused_bus()
        bad = copy.deepcopy(bus.packet.state)
        bad['torque'] = bad['torque'][:15]
        with self.assertRaisesRegex(RuntimeError, 'incomplete16'):
            self.resume(bus, [(bad, 0, 0, {})])

    def test_operator_cancel_exits_pause_and_allows_cleanup(self):
        bus = paused_bus()
        bus.cancel_callback = lambda:(_ for _ in ()).throw(pause.pose.OperatorStop())
        with self.assertRaises(pause.pose.OperatorStop):
            self.resume(bus, [])
        self.assertFalse(bus._motion_paused)
        bus.cancel_callback = lambda:None
        bus.off()
        self.assertTrue(bus.report['all_torque_off_verified'])

    def test_same_index_and_write_count_are_checked(self):
        for change in ('index', 'writes', 'command'):
            with self.subTest(change=change):
                bus = paused_bus()

                def altered(bus):
                    if change == 'index':
                        bus.report['completed_goals'] += 1
                    elif change == 'writes':
                        bus.report.setdefault('writes', []).append({'address':116})
                    else:
                        bus.last_goal_command = bus.last_goal_command+1
                    return bus.packet.state, 0, 0, {}

                with self.assertRaisesRegex(RuntimeError, 'changed during READ-only pause'):
                    self.resume(bus, [altered])

    def test_register_writes_are_prohibited_while_paused(self):
        bus = paused_bus()
        bus._motion_paused = True
        for address in (64, 98, 102, 116):
            with self.assertRaisesRegex(RuntimeError, 'prohibited'):
                bus.write(address, 1, np.ones(16))
        self.assertEqual(bus.packet.calls, [])

    def test_raw_sdk_write_is_blocked_during_pause_but_READ_is_allowed(self):
        from types import SimpleNamespace
        sent = []

        def constructor(bus, report, cancel, **kwargs):
            bus.port = SimpleNamespace(writePort=lambda data:sent.append(data))

        with patch.object(pause.RecoveryBus, '__init__', constructor):
            bus = pause.PauseBus({}, lambda:None)
        bus._motion_paused = True
        for instruction in (3, 0x83, 8):
            with self.assertRaisesRegex(RuntimeError, 'prohibited'):
                bus.port.writePort([255, 255, 253, 0, 1, 3, 0, instruction])
        read = [255, 255, 253, 0, 1, 3, 0, 2]
        bus.port.writePort(read)
        self.assertEqual(sent, [read])

    def test_real_controller_loop_resumes_at_next_unsent_row_without_duplicate_or_skip(self):
        from test_measured_grip_hold_20261117 import FakeBus, state, Clock as LoopClock
        initial = state(1)
        initial['position'] = initial['goal'].copy()
        initial['watchdog'][:] = 25

        class LoopBus(pause.PauseBus):
            def __init__(self):
                self.__dict__.update(FakeBus(initial).__dict__)
                self.read_only = self._motion_paused = self._shutdown_started = False
                self.fault_injected = False
                self.pause_reads = 0

            def snapshot(self):
                if self.report.get('completed_goals') == 2 and not self.fault_injected:
                    self.fault_injected = True
                    raise pause.LinkReadFailure('Injected timeout at row2')
                return FakeBus.snapshot(self)

            def write(self, address, size, values):
                if self._motion_paused:
                    raise AssertionError('Controller wrote during pause')
                return FakeBus.write(self, address, size, values)

            def off(self):
                return FakeBus.off(self)

            def pause_probe(self, method, deadline):
                self.pause_reads += 1
                self.assert_no_index_change()
                return FakeBus.snapshot(self), 0, 0, {}

            def assert_no_index_change(self):
                if self.report['completed_goals'] != 2:
                    raise AssertionError('Index advanced without a complete health state')

        bus = LoopBus()
        clock = LoopClock()
        rows = np.tile(initial['goal'], (4, 1))
        rows[:, 0] += np.arange(1, 5)

        def cancel():
            if bus.report.get('holding'):
                raise pause.pose.OperatorStop()

        with patch.object(pause.calibrated, 'sample', pause.sample), \
                patch('combined_quiet_reserve_20261029.quiet'), \
                contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(pause.pose.OperatorStop):
                pause.pose.run_pose(bus, initial, initial['goal'].copy(), rows, bus.report,
                    cancel, now=clock.now, sleep=clock.sleep, limit=float('inf'))
        goals = np.asarray([values for address, values in bus.writes if address == 116])
        np.testing.assert_array_equal(goals, rows)
        self.assertEqual(bus.pause_reads, 2)
        self.assertEqual(bus.report['completed_goals'], 4)
        self.assertTrue(bus.report['all_torque_off_verified'])
        self.assertTrue(bus.report['settings_restored'])

    def test_only_transport_failure_is_wrapped_for_admitted_motion(self):
        bus = paused_bus()

        def timeout(self):
            self.report.setdefault('read_failures', []).append(dict(comm=-3001, device_error=0))
            raise RuntimeError('Full16 health recovery: comm=-3001, error=0; no target advance')

        with patch.object(pause.RecoveryBus, 'snapshot', timeout):
            with self.assertRaises(pause.LinkReadFailure):
                bus.snapshot()
            del bus.report['accepted_start_goal']
            with self.assertRaises(RuntimeError) as raised:
                bus.snapshot()
            self.assertNotIsInstance(raised.exception, pause.LinkReadFailure)
        with patch.object(pause.RecoveryBus, 'snapshot', side_effect=RuntimeError('Motor state changed')):
            with self.assertRaisesRegex(RuntimeError, 'Motor state changed') as raised:
                bus.snapshot()
            self.assertNotIsInstance(raised.exception, pause.LinkReadFailure)

    def test_sample_preserves_normal_path_and_only_catches_typed_link_cause(self):
        bus = paused_bus()
        clean = bus.packet.state
        with patch.object(pause, 'FAST_SAMPLE', return_value=(clean, np.zeros(16))), \
                patch.object(pause, 'resume_health') as recover:
            pause.sample(bus, bus.last_goal_command, bus.report, 'normal')
            recover.assert_not_called()
        with patch.object(pause, 'FAST_SAMPLE', side_effect=RuntimeError('Overcurrent')), \
                patch.object(pause, 'resume_health') as recover:
            with self.assertRaisesRegex(RuntimeError, 'Overcurrent'):
                pause.sample(bus, bus.last_goal_command, bus.report, 'normal')
            recover.assert_not_called()
        try:
            try:
                raise pause.LinkReadFailure('timeout')
            except pause.LinkReadFailure as cause:
                raise RuntimeError('Fresh-state confirmation failed') from cause
        except RuntimeError as wrapped:
            with patch.object(pause, 'FAST_SAMPLE', side_effect=wrapped), \
                    patch.object(pause, 'resume_health', return_value=(clean, np.zeros(16))) as recover:
                pause.sample(bus, bus.last_goal_command, bus.report, 'same_index')
                recover.assert_called_once()

    def test_new_runner_without_execute_opens_no_devices_and_restores_factories(self):
        import leap_vertical_direct_grip_resume_20261125 as runner
        original_bus = runner.base.RecoveryBus
        original_sample = runner.calibrated.sample
        with patch.object(runner, 'validate_sources'), \
                patch.object(pause.PauseBus, '__init__', side_effect=AssertionError('No devices')), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(runner.main([]), 0)
        self.assertIs(runner.base.RecoveryBus, original_bus)
        self.assertIs(runner.calibrated.sample, original_sample)


if __name__ == '__main__':
    unittest.main(verbosity=2)
