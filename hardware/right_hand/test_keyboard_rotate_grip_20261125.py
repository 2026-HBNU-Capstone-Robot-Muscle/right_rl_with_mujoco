"""Keyboard rotation boundaries and one-session controller tests, fake I/O."""
import contextlib
import copy
import io
import json
import unittest
from unittest.mock import patch

import numpy as np

import calibrated_two_full_health_20261028 as calibrated
import goal_readback_confirmation as readback
import grip_pause_resume_10s_trial_20261125 as recovery
import leap_keyboard_rotate_grip_20261125 as app
import leap_measured_grip_hold_20261117 as pose
from test_measured_grip_hold_20261117 import Clock, state
from test_rotation_speed_resume_20261125 import Bus
from vertical_speed_trial_plan_20261125 import ORIGINAL_VALIDATOR


class Tests(unittest.TestCase):
    def sequence(self):
        report = {}
        sequence = app.KeyboardRotationSequence(report)
        initial = state(1)
        initial['goal'] = initial['position'] = sequence.start.copy()
        initial['watchdog'][:] = 25
        bus = Bus(initial)
        bus.report = report
        sequence.bus = bus
        sequence.rows(sequence.start)
        return sequence, bus, initial

    def test_three_fresh_checked_G_samples_are_required_and_duplicates_do_not_count(self):
        sequence, bus, initial = self.sequence()
        self.assertFalse(sequence.request_axis('horizontal'))
        bus.report['phase'] = app.G_HOLD
        with contextlib.redirect_stdout(io.StringIO()):
            for i in range(2):
                bus.last_reviewed_state = copy.deepcopy(initial)
                sequence.phase(len(sequence.actual_rows), False)
            sequence.phase(len(sequence.actual_rows), False)
            self.assertEqual(sequence.confirmed, 2)
            self.assertFalse(sequence.ready_for_input)
            bad = copy.deepcopy(initial)
            bad['goal'][0] += 1
            bus.last_reviewed_state = bad
            sequence.phase(len(sequence.actual_rows), False)
            self.assertEqual(sequence.confirmed, 0)
            for _ in range(3):
                bus.last_reviewed_state = copy.deepcopy(initial)
                sequence.phase(len(sequence.actual_rows), False)
        self.assertTrue(sequence.ready_for_input)

    def test_keyboard_mapping_busy_keys_and_recovery_do_not_queue_extra_motion(self):
        sequence, bus, _ = self.sequence()
        terminal = app.KeyboardTerminal(sequence, type('Stop', (), {'check':lambda self:None})())
        self.assertFalse(terminal.handle('A'))
        sequence.ready_for_input = True
        bus._motion_paused = True
        self.assertFalse(terminal.handle('B'))
        bus._motion_paused = False
        self.assertTrue(terminal.handle('A'))
        self.assertEqual(sequence.pending, 'horizontal')
        self.assertFalse(terminal.handle('b'))
        self.assertEqual(sequence.pending, 'horizontal')
        for key in (' ', 'q', 'Q', '\x1b', '\x03'):
            with self.assertRaises(pose.OperatorStop):
                terminal.handle(key)
        sequence.pending = None
        sequence.ready_for_input = True
        self.assertTrue(terminal.handle('b'))
        self.assertEqual(sequence.pending, 'vertical')

    def run_commands(self, axes, *, fault=False, bad_confirmation=False):
        sequence, bus, _ = self.sequence()
        initial = state(0)
        initial['goal'] = initial['position'] = sequence.start.copy()
        bus = Bus(initial, fault=fault, bad_confirmation=bad_confirmation)
        bus.report = sequence.report
        sequence.bus = bus
        self.sequence_used, self.bus_used = sequence, bus
        rows = sequence.rows(sequence.start)
        sent = []
        clock = Clock()

        def cancel():
            if sequence.ready_for_input:
                completed = sequence.report.get('completed_commands', [])
                if len(completed) == len(axes):
                    raise pose.OperatorStop()
                if len(sent) < len(axes):
                    axis = axes[len(sent)]
                    self.assertTrue(sequence.request_axis(axis))
                    sent.append(axis)

        with sequence.runtime_scope(), patch.object(calibrated, 'sample', recovery.sample), \
                patch('combined_quiet_reserve_20261029.quiet'), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(pose.OperatorStop):
                pose.run_pose(bus, initial, sequence.start.copy(), rows, bus.report, cancel,
                    sequence=sequence, limit=float('inf'), now=clock.now, sleep=clock.sleep)
        return sequence, bus

    def test_A_B_A_runs_exact_candidates_in_one_enable_session_without_intermediate_OFF(self):
        axes = ['horizontal', 'vertical', 'horizontal']
        sequence, bus = self.run_commands(axes)
        np.testing.assert_array_equal([q for _,q in bus.goals[1:]], sequence.actual_rows)
        self.assertEqual(bus.report['completed_goals'], len(sequence.actual_rows))
        self.assertEqual([c['axis'] for c in bus.report['completed_commands']], axes)
        for command in bus.report['completed_commands']:
            a, b = command['start_index'], command['end_index']
            np.testing.assert_array_equal(sequence.actual_rows[a:b], sequence.compiled[command['axis']]['rows'])
            self.assertEqual(command['checked_G_samples'], 3)
        self.assertEqual(sum(a==64 and np.all(v) for a,v in bus.writes), 1)
        off_indices = [i for i,(a,v) in enumerate(bus.writes) if a==64 and not np.any(v)]
        last_goal = max(i for i,(a,_) in enumerate(bus.writes) if a==116)
        self.assertTrue(off_indices and all(i>last_goal for i in off_indices))
        self.assertTrue(bus.report['all_torque_off_verified'])
        self.assertTrue(bus.report['settings_restored'])
        self.assertIs(pose.bounds, app.ORIGINAL_BOUNDS)
        self.assertIs(pose.validate_goal_return, ORIGINAL_VALIDATOR)
        self.assertEqual(readback.MAX_STEP_TICKS, 2)

    def test_READ_recovery_does_not_duplicate_or_omit_targets_in_keyboard_motion(self):
        sequence, bus = self.run_commands(['horizontal'], fault=True)
        self.assertTrue(bus.injected)
        self.assertEqual(bus.report['communication_pause_events'][0]['status'], 'resumed')
        np.testing.assert_array_equal([q for _,q in bus.goals[1:]], sequence.actual_rows)
        self.assertEqual(len(bus.report['completed_commands']), 1)

    def test_bad_recovery_state_stops_and_cleans_up_instead_of_accepting_next_axis(self):
        with self.assertRaisesRegex(RuntimeError, 'current'):
            self.run_commands(['horizontal', 'vertical'], fault=True, bad_confirmation=True)
        self.assertEqual([c['axis'] for c in self.bus_used.report['requested_commands']], ['horizontal'])
        self.assertEqual(self.bus_used.report.get('completed_commands'), [])
        self.assertTrue(self.bus_used.report['all_torque_off_verified'])
        self.assertTrue(self.bus_used.report['settings_restored'])
        self.assertIs(pose.bounds, app.ORIGINAL_BOUNDS)
        self.assertIs(pose.validate_goal_return, ORIGINAL_VALIDATOR)
        self.assertEqual(readback.MAX_STEP_TICKS, 2)

    def test_default_preview_opens_no_devices(self):
        with patch.object(app, 'PauseBus', side_effect=AssertionError('No hardware')), \
                contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(app.main([]), 0)
        data = json.loads(output.getvalue())
        self.assertFalse(data['devices_opened'])
        self.assertEqual(data['keyboard_mapping'], {'a':'horizontal', 'b':'vertical'})


if __name__ == '__main__':
    unittest.main()
