"""Separate3x paths and pause interaction, with fake I/O only."""
import contextlib
import copy
import io
import json
import time
import unittest
from unittest.mock import patch

import numpy as np

import calibrated_two_full_health_20261028 as calibrated
import goal_readback_confirmation as readback
import grip_pause_resume_10s_trial_20261125 as recovery
import sequence_flip_checked as checked
import leap_measured_grip_hold_20261117 as pose
import leap_rotation_speed_resume_20261125 as runner
from rotation_speed_resume_plan_20261125 import (BASE, RotationSpeedTrial, ORIGINAL_BOUNDS,
    SLOW_SEATING_PHASES, horizontal_middle_index_support, horizontal_thumb_first_return,
    horizontal_tips_before_lower_push, TIP_CURL_PHASE, EXTRA_TIP_CURL_TICKS,
    MIDDLE_ROOT_LOWER_TICKS, index_first_then_middle,
    INDEX_FIRST_PHASES, VERTICAL_INDEX_FIRST_PHASES)
from test_measured_grip_hold_20261117 import Clock, FakeBus, state
from vertical_speed_trial_plan_20261125 import ORIGINAL_VALIDATOR


class Bus(recovery.PauseBus):
    def __init__(self, initial, *, fault=False, bad_confirmation=False):
        self.__dict__.update(FakeBus(initial).__dict__)
        self.goals = []
        self.read_only = self._motion_paused = self._shutdown_started = self._using_unicast = False
        self.fault, self.injected, self.bad_confirmation = fault, False, bad_confirmation
        self.probe_reads = 0

    def snapshot(self):
        if (self.fault and not self.injected and np.any(self.state['torque'])
                and self.report.get('phase', '').startswith('trial_')
                and self.report.get('completed_goals', 0) > 60):
            self.injected = True
            raise recovery.LinkReadFailure('Injected READ timeout in fast motion')
        return FakeBus.snapshot(self)

    def pause_probe(self, method, deadline):
        self.probe_reads += 1
        value = FakeBus.snapshot(self)
        if self.bad_confirmation and self.probe_reads == 2:
            value['current'][0] = 600
        return value, 0, 0, {}

    def write(self, address, size, values):
        if self._motion_paused:
            raise AssertionError('Motor register write during pause')
        if address == 116:
            values = pose.validate_goal_return(values, self.last_goal_command, self.entry_anchor)
            self.goals.append((self.report.get('phase', ''), values.copy()))
        return FakeBus.write(self, address, size, values)

    def off(self):
        return FakeBus.off(self)


class Tests(unittest.TestCase):
    def slow_sample_bus(self):
        initial = state(1)
        initial['position'] = initial['goal'].copy()
        initial['watchdog'][:] = 25
        bus = Bus(initial)
        bus.controlled = True
        bus.report = dict(enabled_at=time.monotonic(), completed_goals=2981,
                          accepted_start_goal=initial['goal'].tolist())
        bus.last_goal_command = initial['goal'].copy()
        bus.last_reviewed_state = copy.deepcopy(initial)
        bus.checked_control_profile = dict(caps=initial['current_cap'],
            gains={k:initial[k] for k in ('p_gain','i_gain','d_gain')})
        return bus

    def run_slow_sample(self, bus, *, snapshot_check=None, started_voltage_confirmation=False):
        def snapshot():
            value = FakeBus.snapshot(bus)
            bus._last_complete_snapshot = value
            return value

        def delayed_sample(bus, command, report, phase, **kwargs):
            if started_voltage_confirmation:
                report.setdefault('voltage_events', []).append(dict(status='confirming'))
            base = time.monotonic()
            with patch.object(bus, 'snapshot', side_effect=snapshot), \
                    patch.object(checked.time, 'monotonic', side_effect=[base,base+.203,base+.203]):
                # Exercise the exact source timing trip, not a fabricated
                # RuntimeError in place of the original sampler.
                return checked._read_sample(bus, command, report, phase, kwargs.get('snapshot_check'))

        with patch.object(recovery, 'FAST_SAMPLE', side_effect=delayed_sample), \
                patch('combined_quiet_reserve_20261029.quiet'), contextlib.redirect_stdout(io.StringIO()):
            return recovery.sample(bus, bus.last_goal_command, bus.report, 'trial_vertical_return',
                                   snapshot_check=snapshot_check)

    def test_snapshot_retains_complete_packet_without_marking_it_reviewed(self):
        bus = object.__new__(recovery.PauseBus)
        bus.report = {}
        value = state(1)
        with patch.object(recovery.RecoveryBus, 'snapshot', return_value=value) as parent:
            self.assertIs(bus.snapshot(), value)
        parent.assert_called_once()
        self.assertIs(bus._last_complete_snapshot, value)
        self.assertFalse(hasattr(bus, 'last_reviewed_state'))

    def test_real203ms_sample_rechecks_two_new_states_without_goal_or_register_writes(self):
        bus = self.slow_sample_bus()
        goal = bus.last_goal_command.copy()
        self.run_slow_sample(bus)
        event = bus.report['communication_pause_events'][0]
        self.assertEqual((event['status'],event['clean_reads']), ('resumed',2))
        self.assertEqual(bus.probe_reads,2)
        self.assertEqual(bus.report['samples'][0]['sample_role'], 'late_complete_READ_not_used_for_motion')
        self.assertAlmostEqual(bus.report['samples'][0]['read_ms'],203.,places=4)
        self.assertEqual(bus.report['completed_goals'],2981)
        self.assertEqual(bus.writes,[])
        np.testing.assert_array_equal(bus.last_goal_command,goal)
        self.assertFalse(bus._motion_paused)

    def test_slow_sample_motor_health_and_held_joint_faults_are_not_transport_retries(self):
        for field,value in [('current',600),('torque',0),('watchdog',0),('voltage',36),('p_gain',123)]:
            with self.subTest(field=field):
                bus = self.slow_sample_bus()
                bus.state[field][0] = value
                with self.assertRaises(RuntimeError):
                    self.run_slow_sample(bus)
                self.assertEqual(bus.probe_reads,0)
                self.assertEqual(bus.writes,[])
        bus = self.slow_sample_bus()
        def held_check(fresh):
            raise RuntimeError('Injected held-joint fault')
        with self.assertRaisesRegex(RuntimeError,'held-joint'):
            self.run_slow_sample(bus,snapshot_check=held_check)
        self.assertEqual(bus.probe_reads,0)

    def test_slow_read_recovery_requires_recorded_packet_and_admitted_active_bus(self):
        bus = self.slow_sample_bus()
        with patch.object(recovery, 'FAST_SAMPLE', side_effect=RuntimeError('Motor read took 203ms; no target advance')):
            with self.assertRaisesRegex(RuntimeError,'Motor read took'):
                recovery.sample(bus,bus.last_goal_command,bus.report,'trial_vertical_return')
        self.assertEqual(bus.probe_reads,0)
        for field,value in [('controlled',False),('read_only',True),('_shutdown_started',True)]:
            with self.subTest(field=field):
                bus = self.slow_sample_bus()
                setattr(bus,field,value)
                with self.assertRaisesRegex(RuntimeError,'Motor read took'):
                    self.run_slow_sample(bus)
                self.assertEqual(bus.probe_reads,0)

    def test_slow_read_bad_fresh_confirmation_still_stops_without_writes(self):
        bus = self.slow_sample_bus()
        bus.bad_confirmation = True
        with self.assertRaisesRegex(RuntimeError,'current'):
            self.run_slow_sample(bus)
        self.assertEqual(bus.report['communication_pause_events'][0]['status'],'stopped')
        self.assertEqual(bus.probe_reads,2)
        self.assertEqual(bus.writes,[])
        self.assertFalse(bus._motion_paused)

    def test_slow_read_cannot_extend_a_started_voltage_confirmation(self):
        bus = self.slow_sample_bus()
        with self.assertRaisesRegex(RuntimeError,'Motor read took'):
            self.run_slow_sample(bus,started_voltage_confirmation=True)
        self.assertEqual(bus.probe_reads,0)
        self.assertEqual(bus.writes,[])

    def test_slow_read_recovery_still_honors_operator_cancel(self):
        bus = self.slow_sample_bus()
        def cancel():
            if bus._motion_paused:
                raise pose.OperatorStop()
        bus.cancel_callback = cancel
        with self.assertRaises(pose.OperatorStop):
            self.run_slow_sample(bus)
        self.assertFalse(bus._motion_paused)
        self.assertEqual(bus.writes,[])

    def ten_second_probe(self, *, recover_after=None, bad_state=False, cancel_after=None):
        initial = state(1)
        initial['watchdog'][:] = 25
        bus = Bus(initial)
        bus.report = dict(enabled_at=time.monotonic(), completed_goals=1020)
        bus.last_goal_command = initial['goal'].copy()
        bus.last_reviewed_state = copy.deepcopy(initial)
        bus.checked_control_profile = dict(caps=initial['current_cap'],
            gains={k:initial[k] for k in ('p_gain','i_gain','d_gain')})
        clock = Clock()
        calls = []
        self.bus = bus

        def probe(method, deadline):
            calls.append(method)
            if recover_after is not None and len(calls) > recover_after:
                clock.t += .03
                value = copy.deepcopy(initial)
                if bad_state:
                    value['torque'][0] = value['watchdog'][0] = 0
                return value, 0, 0, {}
            clock.t += .19
            return None, -3001, 0, {}

        def cancel():
            if cancel_after is not None and clock.t >= cancel_after:
                raise pose.OperatorStop()

        bus.cancel_callback = cancel
        with patch.object(bus, 'pause_probe', side_effect=probe), \
                patch('combined_quiet_reserve_20261029.quiet'), \
                contextlib.redirect_stdout(io.StringIO()):
            return recovery.resume_health(bus, bus.last_goal_command, bus.report, 'trial',
                recovery.LinkReadFailure('Injected missing response'), now=clock.now,
                sleep=clock.sleep)

    def test_10s_wait_can_recover_after_more_than_one_second_without_writes(self):
        self.ten_second_probe(recover_after=15)
        event = self.bus.report['communication_pause_events'][0]
        self.assertEqual(event['status'], 'resumed')
        self.assertEqual(event['maximum_pause_s'], 10.)
        self.assertGreater(event['elapsed_ms'], 1000)
        self.assertLess(event['elapsed_ms'], 10000)
        self.assertEqual(event['clean_reads'], 2)
        self.assertEqual(event['completed_goals_before'], event['completed_goals_after'])
        self.assertEqual(self.bus.writes, [])

    def test_10s_silence_stops_and_keeps_original1s_module_unchanged(self):
        import grip_pause_resume_20261125 as original
        with self.assertRaisesRegex(RuntimeError, 'within10s/240requests'):
            self.ten_second_probe()
        self.assertEqual(self.bus.report['communication_pause_events'][0]['status'], 'stopped')
        self.assertGreaterEqual(self.bus.report['communication_pause_events'][0]['elapsed_ms'], 10000)
        self.assertEqual(original.POLICY['maximum_extra_pause_s'], 1.)
        self.assertEqual(original.POLICY['maximum_requests'], 24)
        self.assertFalse(self.bus._motion_paused)

    def test_10s_wait_still_stops_immediately_if_torque_or_watchdog_changed(self):
        with self.assertRaisesRegex(RuntimeError, 'Motor state changed'):
            self.ten_second_probe(recover_after=1, bad_state=True)
        event = self.bus.report['communication_pause_events'][0]
        self.assertEqual(len(event['attempts']), 2)
        self.assertLess(event['elapsed_ms'], 1000)
        self.assertEqual(self.bus.writes, [])

    def test_10s_wait_remains_cancellable(self):
        with self.assertRaises(pose.OperatorStop):
            self.ten_second_probe(cancel_after=1.5)
        self.assertFalse(self.bus._motion_paused)
        self.assertEqual(self.bus.writes, [])

    def run_loop(self, axis, *, torque=1, fault=False, bad_confirmation=False):
        sequence = RotationSpeedTrial(axis)
        initial = state(torque)
        initial['position'] = initial['goal'].copy()
        bus = Bus(initial, fault=fault, bad_confirmation=bad_confirmation)
        clock = Clock()
        start = initial['goal'].copy()
        rows = sequence.rows(start)
        self.sequence, self.bus = sequence, bus

        def cancel():
            if bus.report.get('holding'):
                raise pose.OperatorStop()

        with sequence.runtime_scope(), patch.object(calibrated, 'sample', recovery.sample), \
                patch('combined_quiet_reserve_20261029.quiet'), \
                contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(pose.OperatorStop):
                pose.run_pose(bus, initial, start, rows, bus.report, cancel,
                    sequence=sequence, limit=float('inf'), now=clock.now, sleep=clock.sleep)
        return sequence, bus

    def test_both_axes_select_from_checked_dense_baseline_and_finish_at_G(self):
        for axis in ('horizontal', 'vertical'):
            with self.subTest(axis=axis):
                sequence = RotationSpeedTrial(axis)
                rows = sequence.rows(sequence.start)
                np.testing.assert_array_equal(rows, sequence.original_rows[sequence.indices])
                with np.load(BASE/axis/'candidate.npz', allow_pickle=False) as z:
                    np.testing.assert_array_equal(rows, z['goals'])
                self.assertTrue(set(sequence.mandatory).issubset(set(sequence.indices)))
                np.testing.assert_array_equal(rows[-1], sequence.start)
                self.assertLessEqual(np.max(abs(np.diff(np.vstack([sequence.start, rows]), axis=0))), 6)
                entry = [i for i,p in enumerate(sequence.labels) if p.startswith('direct_entry_')]
                for i in entry:
                    previous = sequence.indices[i-1] if i else -1
                    self.assertEqual(sequence.indices[i]-previous, 1)
                self.assertTrue(sequence.authorized())
                self.assertEqual(sequence.period(0, True), .075)

    def test_horizontal_thumb_is_lowered_before_seating_and_stays_lowered(self):
        sequence = RotationSpeedTrial('horizontal')
        source = sequence.reference.horizontal
        expected, expected_phases = horizontal_middle_index_support(source.goals, source.phases)
        begin = int(np.flatnonzero(expected_phases == 'return_middle_root_lead')[0])
        labels = sequence.core_phases
        lower = np.flatnonzero(labels == 'return_thumb_lower_before_support')
        np.testing.assert_array_equal(sequence.core[:begin], expected[:begin])
        np.testing.assert_array_equal(labels[:begin], expected_phases[:begin])
        self.assertEqual(int(lower[0]), begin)
        lowering = sequence.core[lower]
        np.testing.assert_array_equal(lowering[:, 4:12],
            np.tile(expected[begin-1, 4:12], (len(lowering), 1)))
        np.testing.assert_array_equal(lowering[-1, :4], sequence.start[:4])
        np.testing.assert_array_equal(lowering[-1, 12:], sequence.start[12:])
        suffix = sequence.core[int(lower[-1])+1:]
        # Finger timing now differs, but every receiving stage still ends at
        # its recorded non-tip target and the lowered thumb/ring never move.
        non_tips = [5, 6, 7, 9, 10, 11]
        for phase in set(expected_phases[begin:]):
            np.testing.assert_array_equal(sequence.core[labels == phase][-1, non_tips],
                expected[expected_phases == phase][-1, non_tips])
        np.testing.assert_array_equal(suffix[:, :4], np.tile(sequence.start[:4], (len(suffix), 1)))
        np.testing.assert_array_equal(suffix[:, 12:], np.tile(sequence.start[12:], (len(suffix), 1)))
        self.assertGreater(int(np.flatnonzero(labels == 'return_grip_index_middle_seat')[0]),
                           int(lower[-1]))
        self.assertLessEqual(np.max(abs(np.diff(np.vstack([sequence.start, sequence.core]), axis=0))), 2)

    def test_middle_matches_index_bends_without_overwithdrawal_or_later_root_drop(self):
        sequence = RotationSpeedTrial('horizontal')
        source = sequence.reference.horizontal
        rows, phases = horizontal_middle_index_support(source.goals, source.phases)
        begin = int(np.flatnonzero(source.phases == 'return_transfer_middle_clear')[0])
        np.testing.assert_array_equal(rows[:begin], source.goals[:begin])
        np.testing.assert_array_equal(phases[:begin], source.phases[:begin])
        expected = source.goals[begin-1, [4, 6, 7]].copy()
        expected[2] -= MIDDLE_ROOT_LOWER_TICKS
        np.testing.assert_array_equal(expected, [2005, 1843, 2002])
        fixed = np.r_[0:8, 12:16]
        clear = rows[phases == 'return_transfer_middle_clear']
        np.testing.assert_array_equal(clear[:, fixed],
            np.tile(source.goals[begin-1, fixed], (len(clear), 1)))
        np.testing.assert_array_equal(clear[-1, [8, 10, 11]], expected)
        self.assertEqual(clear[-1, 9], source.goals[source.phases == 'return_transfer_middle_clear'][-1, 9])
        for name in ('return_transfer_thumb_prepare', 'return_transfer_thumb_push'):
            stage = rows[phases == name]
            np.testing.assert_array_equal(stage[:, [8, 10, 11]], np.tile(expected, (len(stage), 1)))
            old_end = source.goals[source.phases == name][-1]
            np.testing.assert_array_equal(stage[-1, fixed], old_end[fixed])
        tips = rows[phases == 'return_transfer_thumb_wrist_and_tips'][-1]
        self.assertEqual(tips[8], source.goals[source.phases == 'return_transfer_thumb_wrist_and_tips'][-1, 8])
        np.testing.assert_array_equal(tips[[10, 11]], expected[[1, 2]])
        clear_end = int(np.flatnonzero(phases == 'return_transfer_middle_clear')[-1])
        self.assertGreaterEqual(int(rows[clear_end:, 11].min()), int(expected[2]))
        np.testing.assert_array_equal(rows[-1], sequence.start)
        self.assertLessEqual(np.max(abs(np.diff(np.vstack([sequence.start, rows]), axis=0))), 2)

    def test_four_degree_lowering_changes_only_middle_root_in_dense_path(self):
        sequence = RotationSpeedTrial('horizontal')
        from build_vertical_direct_grip_20261118 import grip_entry
        source = sequence.reference.horizontal
        core, labels = horizontal_middle_index_support(source.goals, source.phases)
        core, labels = horizontal_thumb_first_return(core, labels, sequence.start)
        core, labels = horizontal_tips_before_lower_push(core, labels)
        intro, _ = grip_entry(sequence.start, sequence.start)
        with np.load(BASE/'before_middle_root_lower_4deg/horizontal/candidate.npz', allow_pickle=False) as z:
            before = z['baseline_goals']
        # Isolate the4degree edit before the later intentional timing change.
        after = np.vstack([intro, np.tile(sequence.start, (40,1)), sequence.start, core, sequence.start])
        other = [i for i in range(16) if i != 11]
        np.testing.assert_array_equal(after[:, other], before[:, other])
        root_delta = after[:, 11]-before[:, 11]
        self.assertEqual(int(root_delta.min()), -46)
        self.assertEqual(int(root_delta.max()), 0)
        self.assertAlmostEqual(MIDDLE_ROOT_LOWER_TICKS*360/4096, 4., delta=.05)
        np.testing.assert_array_equal(after[-1], sequence.start)

    def test_only_tips_curl_more_before_lower_push_and_return_smoothly_to_G(self):
        sequence = RotationSpeedTrial('horizontal')
        source = sequence.reference.horizontal
        before, phases = horizontal_middle_index_support(source.goals, source.phases)
        before, phases = horizontal_thumb_first_return(before, phases, sequence.start)
        rows, labels = horizontal_tips_before_lower_push(before, phases)
        followed, followed_phases = index_first_then_middle(rows, labels)
        np.testing.assert_array_equal(followed, sequence.core)
        np.testing.assert_array_equal(followed_phases, sequence.core_phases)
        begin = int(np.flatnonzero(phases == 'return_grip_lower_push')[0])
        np.testing.assert_array_equal(rows[:begin], before[:begin])
        lower_end = int(np.flatnonzero(labels == 'return_thumb_lower_before_support')[-1])
        prepare = np.flatnonzero(labels == TIP_CURL_PHASE)
        self.assertGreater(int(prepare[0]), lower_end)
        tips, other = [4, 8], [i for i in range(16) if i not in (4, 8)]
        np.testing.assert_array_equal(rows[prepare][:, other],
            np.tile(before[begin-1, other], (len(prepare), 1)))
        np.testing.assert_array_equal(rows[prepare[-1], tips], [3251, 3176])
        self.assertEqual(EXTRA_TIP_CURL_TICKS, 64)
        for phase in ('return_grip_lower_push', 'return_fixed_grip_close'):
            old, new = before[phases == phase], rows[labels == phase]
            np.testing.assert_array_equal(new[:, other], old[:, other])
            if phase == 'return_grip_lower_push':
                np.testing.assert_array_equal(new[:, tips], old[:, tips]+64)
        np.testing.assert_array_equal(rows[-1], sequence.start)
        self.assertLessEqual(np.max(abs(np.diff(np.vstack([sequence.start, rows]), axis=0))), 2)

    def test_index_finishes_before_middle_moves_on_identical_joint_paths(self):
        sequence = RotationSpeedTrial('horizontal')
        source = sequence.reference.horizontal
        before, phases = horizontal_middle_index_support(source.goals, source.phases)
        before, phases = horizontal_thumb_first_return(before, phases, sequence.start)
        before, phases = horizontal_tips_before_lower_push(before, phases)
        self.assert_index_first_paths(sequence, before, phases)

    def assert_index_first_paths(self, sequence, before, phases):
        after, labels = index_first_then_middle(before, phases,
            phases_to_split=sequence.index_first_phases)
        np.testing.assert_array_equal(after, sequence.core)
        np.testing.assert_array_equal(labels, sequence.core_phases)
        begin = int(np.flatnonzero(phases == sequence.index_first_phases[0])[0])
        np.testing.assert_array_equal(after[:begin], before[:begin])
        for phase in sequence.index_first_phases:
            old = before[phases == phase]
            delay = len(old)
            new = after[np.isin(labels, [phase+'_index_lead', phase])]
            start = before[int(np.flatnonzero(phases == phase)[0])-1]
            np.testing.assert_array_equal(new[:delay, 8:12], np.tile(start[8:12], (delay,1)))
            np.testing.assert_array_equal(new[delay:, 8:12], old[:, 8:12])
            np.testing.assert_array_equal(new[:len(old), 4:8], old[:, 4:8])
            np.testing.assert_array_equal(new[len(old):, 4:8], np.tile(old[-1,4:8], (delay,1)))
            self.assertTrue(np.any(new[:delay,4:8] != start[4:8]))
            steps = np.diff(np.vstack([start, new]), axis=0)
            both = np.any(steps[:,4:8] != 0, axis=1) & np.any(steps[:,8:12] != 0, axis=1)
            self.assertFalse(np.any(both), 'Index and middle must not move together in a seating stage')
            np.testing.assert_array_equal(new[-1], old[-1])
            np.testing.assert_array_equal(new[:,:4], np.tile(sequence.start[:4], (len(new),1)))
            np.testing.assert_array_equal(new[:,12:], np.tile(sequence.start[12:], (len(new),1)))
        np.testing.assert_array_equal(after[-1], sequence.start)
        self.assertLessEqual(np.max(abs(np.diff(np.vstack([sequence.start, after]),axis=0))),2)

    def test_vertical_index_finishes_before_middle_even_after_fast_selection(self):
        sequence = RotationSpeedTrial('vertical')
        source = sequence.reference.vertical
        self.assert_index_first_paths(sequence, source.goals, source.phases)
        rows = sequence.rows(sequence.start)
        labels = np.asarray(sequence.labels)
        steps = np.diff(np.vstack([sequence.start,rows]),axis=0)
        for phase in VERTICAL_INDEX_FIRST_PHASES:
            mask = np.isin(labels, ['trial_vertical_'+phase+'_index_lead','trial_vertical_'+phase])
            both = np.any(steps[mask,4:8] != 0,axis=1) & np.any(steps[mask,8:12] != 0,axis=1)
            self.assertFalse(np.any(both), 'Fast source selection must not combine finger movement')

    def test_vertical_rotation_prefix_stage_ends_and_thumb_first_entry_are_preserved(self):
        sequence = RotationSpeedTrial('vertical')
        source = sequence.reference.vertical
        begin = int(np.flatnonzero(source.phases == VERTICAL_INDEX_FIRST_PHASES[0])[0])
        np.testing.assert_array_equal(sequence.core[:begin], source.goals[:begin])
        np.testing.assert_array_equal(sequence.core_phases[:begin], source.phases[:begin])
        for phase in VERTICAL_INDEX_FIRST_PHASES:
            np.testing.assert_array_equal(sequence.core[sequence.core_phases == phase][-1],
                source.goals[source.phases == phase][-1])
        current = sequence.start.copy()
        current[:4] -= 20
        rows = sequence.rows(current)
        lower = np.flatnonzero(np.asarray(sequence.labels) == 'direct_entry_thumb_lower')
        np.testing.assert_array_equal(rows[lower, 4:], np.tile(current[4:], (len(lower), 1)))
        np.testing.assert_array_equal(rows[lower[-1], :4], sequence.start[:4])

    def test_final_receiving_and_closing_keep_every_original_two_tick_goal(self):
        sequence = RotationSpeedTrial('horizontal')
        rows = sequence.rows(sequence.start)
        seen = set()
        for i, phase in enumerate(sequence.labels):
            if phase in SLOW_SEATING_PHASES:
                seen.add(phase)
                previous = sequence.indices[i-1] if i else -1
                self.assertEqual(sequence.indices[i]-previous, 1)
                sequence.phase(i, True)
                self.assertEqual(sequence._limit(), 2)
                self.assertLessEqual(np.max(abs(rows[i]-(rows[i-1] if i else sequence.start))), 2)
        self.assertEqual(seen, set(SLOW_SEATING_PHASES))

    def test_full_paths_use_checked_six_tick_packets_and_restore_original_globals(self):
        for axis in ('horizontal', 'vertical'):
            with self.subTest(axis=axis):
                sequence, bus = self.run_loop(axis)
                np.testing.assert_array_equal([q for p,q in bus.goals], sequence.actual_rows)
                self.assertEqual(bus.report['completed_goals'], len(sequence.actual_rows))
                self.assertTrue(bus.report['holding'])
                self.assertTrue(bus.report['all_torque_off_verified'])
                self.assertTrue(bus.report['settings_restored'])
                self.assertTrue(any(np.max(abs(b-a)) > 2 for (_,a),(_,b) in zip(bus.goals,bus.goals[1:])))
                self.assertIs(pose.bounds, ORIGINAL_BOUNDS)
                self.assertIs(pose.validate_goal_return, ORIGINAL_VALIDATOR)
                self.assertEqual(readback.MAX_STEP_TICKS, 2)

    def test_OFF_entry_stays_two_ticks_and_has_one_enable(self):
        for axis in ('horizontal', 'vertical'):
            sequence, bus = self.run_loop(axis, torque=0)
            entry = np.array([q for p,q in bus.goals if p.startswith('direct_entry_')])
            self.assertLessEqual(np.max(abs(np.diff(np.vstack([bus.report['accepted_start_goal'],entry]),axis=0))),2)
            self.assertEqual(sum(a==64 and np.all(v) for a,v in bus.writes),1)

    def test_pause_recovery_in_fast_motion_does_not_repeat_or_skip_a_target(self):
        for axis in ('horizontal', 'vertical'):
            sequence, bus = self.run_loop(axis, fault=True)
            self.assertTrue(bus.injected)
            event = bus.report['communication_pause_events'][0]
            self.assertEqual((event['status'], event['clean_reads']), ('resumed', 2))
            self.assertEqual(event['completed_goals_before'], event['completed_goals_after'])
            np.testing.assert_array_equal([q for p,q in bus.goals], sequence.actual_rows)

    def test_bad_recovery_state_stops_fast_motion_and_cleans_up(self):
        with self.assertRaisesRegex(RuntimeError, 'current'):
            self.run_loop('horizontal', fault=True, bad_confirmation=True)
        self.assertTrue(self.bus.report['all_torque_off_verified'])
        self.assertTrue(self.bus.report['settings_restored'])
        self.assertIs(pose.bounds, ORIGINAL_BOUNDS)
        self.assertIs(pose.validate_goal_return, ORIGINAL_VALIDATOR)
        self.assertEqual(readback.MAX_STEP_TICKS, 2)

    def test_wrong_packet_outside_fast_phase_and_seven_ticks_are_rejected(self):
        sequence = RotationSpeedTrial('horizontal')
        rows = sequence.rows(sequence.start)
        steps = np.max(abs(np.diff(np.vstack([sequence.start,rows]),axis=0)),axis=1)
        index = int(np.flatnonzero(steps>2)[0])
        old = rows[index-1]
        sequence.phase(index, True)
        with sequence.runtime_scope():
            np.testing.assert_array_equal(sequence.validate_packet(rows[index],old,sequence.start),rows[index])
            wrong = rows[index].copy()
            wrong[0] += 1
            with self.assertRaises(RuntimeError):
                sequence.validate_packet(wrong, old, sequence.start)
            wrong = old.copy()
            wrong[0] += 7
            with self.assertRaises(RuntimeError):
                sequence.validate_packet(wrong, old, sequence.start)
            sequence.phase(0, True)
            with self.assertRaises(RuntimeError):
                sequence.validate_packet(rows[index], old, sequence.start)

    def test_default_entries_open_no_devices_and_require_authorization_to_execute(self):
        for axis in ('horizontal', 'vertical'):
            with patch.object(runner, 'PauseBus', side_effect=AssertionError('No hardware')), \
                    contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(runner.main(['--axis',axis]),0)
            self.assertFalse(json.loads(output.getvalue())['devices_opened'])
        with patch.object(RotationSpeedTrial, 'authorized', return_value=False), \
                patch.object(runner, 'PauseBus') as bus, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(runner.main(['--axis','horizontal','--execute']),2)
        bus.assert_not_called()


if __name__ == '__main__':
    unittest.main()
