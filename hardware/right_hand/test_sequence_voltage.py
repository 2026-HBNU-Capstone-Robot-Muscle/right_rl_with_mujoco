"""Offline fault injection: voltage confirmation must never issue a write."""
import contextlib
import copy
import io
import json
import unittest
from unittest.mock import patch
import numpy as np
import sequence_flip_checked as runner
from test_leap_sac_start_empty import FakeBus


def state(voltage=48):
    bus = FakeBus()
    s = bus.snapshot()
    s['torque'][:] = 1; s['watchdog'][:] = 25
    s['current_cap'] = runner.CAP.copy()
    for key, value in runner.GAINS.items():
        s[key] = value.copy()
    s['voltage'][:] = voltage
    return s


class Clock:
    def __init__(self): self.now = 1.
    def monotonic(self): return self.now
    def sleep(self, seconds): self.now += seconds


class ReadOnlyBus:
    def __init__(self, clock, states, read_seconds=.02):
        self.clock = clock; self.states = list(states); self.calls = 0
        self.read_seconds = read_seconds
    def snapshot(self):
        self.clock.now += self.read_seconds
        index = min(self.calls, len(self.states)-1); self.calls += 1
        value = self.states[index]
        if isinstance(value, BaseException): raise value
        return copy.deepcopy(value)
    def write(self, *args, **kwargs):
        raise AssertionError('No write is allowed during voltage confirmation')


class VoltageTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock(); self.report = {'enabled_at': 0.}
        self.command = state()['goal']
        self.stack = contextlib.ExitStack()
        self.stack.enter_context(patch.object(runner.time, 'monotonic', self.clock.monotonic))
        self.stack.enter_context(patch.object(runner.time, 'sleep', self.clock.sleep))
        self.stack.enter_context(patch.object(runner, 'check_cancel'))
        self.stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.addCleanup(self.stack.close)

    def sample(self, states, duration=.02):
        bus = ReadOnlyBus(self.clock, states, duration)
        result = runner.sample(bus, self.command, self.report, 'test')
        return bus, result

    def test_healthy_path_adds_no_reads(self):
        bus, _ = self.sample([state()])
        self.assertEqual(bus.calls, 1)
        self.assertNotIn('voltage_events', self.report)

    def test_dip_recovers_only_after_two_fresh_healthy_reads(self):
        bus, result = self.sample([state(36), state(38), state(39)])
        self.assertEqual(bus.calls, 3)
        self.assertEqual(self.report['voltage_events'][0]['status'], 'recovered')
        self.assertTrue(np.all(result[0]['voltage'] == 39))

    def test_other_motor_dip_resets_recovery_streak(self):
        other = state(48); other['voltage'][15] = 36
        bus, _ = self.sample([state(36), state(38), other, state(38), state(38)])
        self.assertEqual(bus.calls, 5)

    def test_37_is_not_recovery_margin(self):
        with self.assertRaisesRegex(RuntimeError, 'not recovered'):
            self.sample([state(36), state(37)])

    def test_persistent_low_stops_and_is_recorded(self):
        with self.assertRaisesRegex(RuntimeError, 'not recovered'):
            self.sample([state(36)])
        self.assertEqual(self.report['voltage_events'][0]['status'], 'stopped')

    def test_deeper_dip_immediate(self):
        bus = ReadOnlyBus(self.clock, [state(35), state(48)])
        with self.assertRaisesRegex(RuntimeError, 'immediate stop'):
            runner.sample(bus, self.command, self.report, 'test')
        self.assertEqual(bus.calls, 1)

    def test_other_faults_are_never_debounced(self):
        for key, value in [('error', 1), ('torque', 0), ('watchdog', 255),
                           ('current', 551), ('temperature', 46), ('voltage', 66)]:
            with self.subTest(key=key):
                bad = state(48); bad[key][0] = value
                bus = ReadOnlyBus(self.clock, [state(36), bad, state()])
                with self.assertRaises(RuntimeError):
                    runner.sample(bus, self.command, self.report, 'test')
                self.assertLessEqual(bus.calls, 2)
                self.report = {'enabled_at': 0.}

    def test_slow_recovery_read_cannot_resume_after_deadline(self):
        with self.assertRaisesRegex(RuntimeError, 'not recovered'):
            self.sample([state(36), state(48)], duration=.09)

    def test_read_failure_during_confirmation_propagates(self):
        with self.assertRaisesRegex(RuntimeError, 'communication failed'):
            self.sample([state(36), RuntimeError('communication failed')])

    def test_user_stop_during_confirmation_propagates(self):
        with self.assertRaises(KeyboardInterrupt):
            self.sample([state(36), KeyboardInterrupt('Q')])
        self.assertEqual(self.report['voltage_events'][0]['status'], 'stopped')

    def test_third_repeated_event_stops(self):
        for _ in range(2): self.sample([state(36), state(38), state(38)])
        with self.assertRaisesRegex(RuntimeError, 'Repeated undervoltage'):
            self.sample([state(36), state(38), state(38)])

    def test_enable_health_remains_strict(self):
        with self.assertRaises(runner.MildUndervoltage):
            runner.health(state(36), self.command)

    def test_actual_failed_snapshot_triggers_confirmation_not_silent_acceptance(self):
        doc = json.loads((runner.ROOT/'sac_start_hardware_audit_result.json').read_text())
        trial = next(t for t in doc['trials'] if t.get('min_voltage_v') == 3.6)
        saved = trial['worst_voltage_sample']
        s = state(); s['voltage'] = np.rint(np.array(saved['voltage_v'])*10).astype(int)
        s['goal'] = np.array(saved['goal']); s['position'] = np.array(saved['actual'])
        s['current'] = np.array(saved['current_ma']) % 65536
        with self.assertRaises(runner.MildUndervoltage): runner.health(s, s['goal'])
        # The original run ends here: it cannot tell whether a later read recovers.

    def test_run_turns_off_and_restores_after_confirmed_failure(self):
        bus = FakeBus(); initial = bus.snapshot()
        def failed_move(bus, command, target, report, phase):
            bus.state['voltage'][:] = 36
            runner.sample(bus, command, report, phase)
        report = {}
        with patch.object(runner, 'move', side_effect=failed_move):
            with self.assertRaisesRegex(RuntimeError, 'not recovered'):
                runner.run(bus, initial, [('test', initial['position'])], report, empty_check=True)
        self.assertTrue(report['all_torque_off_verified'])
        self.assertTrue(report['settings_restored'])
        self.assertTrue(bus.off_called)


if __name__ == '__main__': unittest.main()
