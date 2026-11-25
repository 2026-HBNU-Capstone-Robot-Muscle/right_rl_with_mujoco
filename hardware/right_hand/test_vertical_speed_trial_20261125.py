"""Device-free regression tests for the isolated speed experiment."""
import contextlib
import copy
import io
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np

import goal_readback_confirmation as readback
import leap_measured_grip_hold_20261117 as pose
import vertical_speed_trial_plan_20261125 as fast
from test_measured_grip_hold_20261117 import Clock, FakeBus, state


class ValidatedBus(FakeBus):
    def __init__(self, initial):
        super().__init__(initial)
        self.goals = []

    def write(self, address, size, values):
        if address == 116:
            values = pose.validate_goal_return(values, self.last_goal_command, self.entry_anchor)
            self.goals.append((self.report.get('phase'), values.copy()))
        return super().write(address, size, values)


class SpeedTests(unittest.TestCase):
    def run_fake(self, seq, bus, *, fault_phase=None, stop_phase=None):
        initial = copy.deepcopy(bus.state)
        start = initial['goal'] if np.any(initial['torque']) else initial['position']
        rows = seq.rows(start)
        clock = Clock()
        bus.report['status'] = 'started'

        def cancel():
            if bus.report.get('phase') == fault_phase and fault_phase:
                bus.state['current'][0] = 600
            if bus.report.get('holding') or (stop_phase and bus.report.get('phase') == stop_phase):
                raise pose.OperatorStop()

        with contextlib.redirect_stdout(io.StringIO()), seq.runtime_scope():
            try:
                pose.run_pose(bus, initial, start, rows, bus.report, cancel, sequence=seq,
                              limit=float('inf'), now=clock.now, sleep=clock.sleep)
            except pose.OperatorStop:
                pass
        return clock.t

    def test_all_bound_original_files_still_match(self):
        hashes = json.loads((fast.BASE/'original_source_hashes.json').read_bytes())
        self.assertEqual(len(hashes), 267)
        self.assertEqual([p for p, h in hashes.items() if fast.digest(fast.ROOT/p) != h], [])

    def test_rows_keep_phase_ends_entry_holds_and_exact_final_G(self):
        for start in (fast.VerticalSpeedTrial().start, state(0)['position']):
            seq = fast.VerticalSpeedTrial()
            rows = seq.rows(start)
            original, indices = seq.original_rows, seq.indices
            np.testing.assert_array_equal(rows, original[indices])
            self.assertTrue(set(seq.mandatory).issubset(set(indices)))
            np.testing.assert_array_equal(rows[-1], seq.start)
            self.assertLessEqual(np.max(abs(np.diff(np.vstack([start, rows]), axis=0))), 6)
            for phase in dict.fromkeys(seq.original.labels):
                members = np.flatnonzero(np.asarray(seq.original.labels) == phase)
                self.assertTrue({members[0], members[-1]}.issubset(set(indices)))
                if phase.startswith('direct_entry_'):
                    self.assertTrue(set(members).issubset(set(indices)))
            for left, right in zip(indices[:-1], indices[1:]):
                t = np.arange(right-left+1)[:, None]/(right-left)
                error = abs(original[left:right+1] - (original[left]+t*(original[right]-original[left])))
                self.assertLessEqual(float(error.max()), 1.+1e-9)
            kept = set(indices)
            stationary = np.flatnonzero(np.all(np.diff(np.vstack([start, original]), axis=0) == 0, axis=1))
            self.assertTrue(set(stationary).issubset(kept))
            self.assertEqual(seq.period(0, True), .075)
            self.assertEqual(seq.period(len(rows), False), .075)

    def test_full_cycle_uses_checked_packets_and_never_OFF_between_phases(self):
        seq = fast.VerticalSpeedTrial()
        initial = state(1)
        bus = ValidatedBus(initial)
        self.run_fake(seq, bus)
        np.testing.assert_array_equal([q for p, q in bus.goals], seq.actual_rows)
        self.assertEqual(bus.report['completed_goals'], len(seq.actual_rows))
        self.assertTrue(bus.report['holding'])
        self.assertTrue(bus.report['all_torque_off_verified'])
        self.assertTrue(bus.report['settings_restored'])
        last_goal = max(i for i, (a, _) in enumerate(bus.writes) if a == 116)
        self.assertTrue(all(i > last_goal for i, (a, v) in enumerate(bus.writes) if a == 64 and not np.any(v)))
        self.assertEqual(seq.metadata()['phase_checkpoint_indices'].keys(), seq.original.metadata()['phase_checkpoint_indices'].keys())
        self.assertIs(pose.validate_goal_return, fast.ORIGINAL_VALIDATOR)
        self.assertEqual(readback.MAX_STEP_TICKS, 2)

    def test_OFF_entry_rebuild_keeps_original_two_tick_entry(self):
        seq = fast.VerticalSpeedTrial()
        bus = ValidatedBus(state(0))
        self.run_fake(seq, bus)
        entry = np.array([q for p, q in bus.goals if p and p.startswith('direct_entry_')])
        whole = np.vstack([bus.report['accepted_start_goal'], entry])
        self.assertLessEqual(np.max(abs(np.diff(whole, axis=0))), 2)
        self.assertTrue(bus.report['settings_restored'])

    def test_health_fault_at_receiving_stops_before_thumb_lower(self):
        seq = fast.VerticalSpeedTrial()
        bus = ValidatedBus(state(1))
        with self.assertRaisesRegex(RuntimeError, 'measured current'):
            self.run_fake(seq, bus, fault_phase='direct_receiving_check')
        self.assertFalse(any(p == 'direct_return_thumb_lower_before_support' for p, q in bus.goals))
        self.assertTrue(bus.report['all_torque_off_verified'])
        self.assertTrue(bus.report['settings_restored'])
        self.assertEqual(readback.MAX_STEP_TICKS, 2)

    def test_user_stop_and_unconfirmed_OFF_are_not_reported_as_clean(self):
        seq = fast.VerticalSpeedTrial()
        bus = ValidatedBus(state(1))

        def fail_OFF():
            raise RuntimeError('Injected OFF confirmation failure')

        bus.off = fail_OFF
        self.run_fake(seq, bus, stop_phase='direct_return_final_taught_1')
        self.assertFalse(bus.report.get('all_torque_off_verified', False))
        self.assertFalse(bus.report.get('settings_restored', False))
        self.assertIn('OFF confirmation failure', bus.report['cleanup_error'])
        self.assertIs(pose.validate_goal_return, fast.ORIGINAL_VALIDATOR)

    def test_fast_validator_rejects_wrong_packet_and_outside_phase(self):
        seq = fast.VerticalSpeedTrial()
        rows = seq.rows(seq.start)
        steps = np.max(abs(np.diff(np.vstack([seq.start, rows]), axis=0)), axis=1)
        index = int(np.flatnonzero(steps > 2)[0])
        previous = rows[index-1] if index else seq.start
        seq.phase(index, True)
        np.testing.assert_array_equal(seq.validate_packet(rows[index], previous, seq.start), rows[index])
        wrong = rows[index].copy()
        wrong[0] += 1
        with self.assertRaises(RuntimeError):
            seq.validate_packet(wrong, previous, seq.start)
        seq.phase(index, False)
        with self.assertRaises(RuntimeError):
            seq.validate_packet(rows[index], previous, seq.start)
        seq.phase(0, True)
        with self.assertRaises(RuntimeError):
            seq.validate_packet(rows[index], previous, seq.start)

    def test_readback_six_tick_previous_packet_still_needs_two_exact_fresh_reads(self):
        seq = fast.VerticalSpeedTrial()
        old = seq.start.copy()
        command = old.copy()
        command[0] += 6
        initial = state(1)
        initial['goal'] = old.copy()
        clean = copy.deepcopy(initial)
        clean['goal'] = command.copy()
        bus = type('ReadBus', (), dict(previous_goal_command=old, last_goal_command=command))()
        report = dict(enabled_at=time.monotonic())
        calls = []

        def fresh():
            calls.append(True)
            return clean, [0]*16

        def validate(s):
            if not np.array_equal(s['goal'], command):
                raise readback.GoalReadbackMismatch()

        with seq.runtime_scope(), contextlib.redirect_stdout(io.StringIO()):
            returned, _ = readback.confirm_previous_goal(bus, command, initial, report, 'trial', read_sample=fresh, validate=validate)
            self.assertIs(returned, clean)
            self.assertEqual(len(calls), 2)
            initial['goal'][1] += 1
            with self.assertRaisesRegex(RuntimeError, 'exact preceding'):
                readback.confirm_previous_goal(bus, command, initial, report, 'bad', read_sample=fresh, validate=validate)
            self.assertEqual(len(calls), 2)
        self.assertEqual((readback.MAX_STEP_TICKS, readback.MAX_READS, readback.EXACT_READS), (2, 3, 2))
        self.assertEqual((readback.CONFIRM_SECONDS, readback.REPEAT_WINDOW_SECONDS, readback.REPEAT_TRIP), (.15, 10., 3))

    def test_tampered_candidate_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            (folder/'trial_manifest.json').write_bytes((fast.BASE/'trial_manifest.json').read_bytes())
            (folder/'candidate.npz').write_bytes((fast.BASE/'candidate.npz').read_bytes()+b'changed')
            with patch.object(fast, 'BASE', folder):
                with self.assertRaisesRegex(RuntimeError, 'candidate changed'):
                    fast.VerticalSpeedTrial()

    def test_unconfirmed_six_tick_execution_does_not_open_devices(self):
        import leap_vertical_speed_trial_20261125 as runner
        with patch.object(fast.VerticalSpeedTrial, 'authorized', return_value=False), \
                patch('sys.argv', ['trial', '--execute']), \
                patch.object(pose, 'PoseBus') as bus, \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(runner.main(), 2)
        bus.assert_not_called()


if __name__ == '__main__':
    unittest.main(verbosity=2)
