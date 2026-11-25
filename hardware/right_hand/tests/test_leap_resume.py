"""Offline tests only: fake servo bus, fake clock and fake camera."""
import contextlib
import io
import json
import sys
import unittest
import tempfile
from unittest.mock import Mock, patch
from types import SimpleNamespace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent / 'work'))
from test_hand_start_pose import FakeBus, Clock
from test_held_policy_shadow import Camera
import hand12_posture_target_100 as motion
import held_policy_shadow as held
import encoder_policy_shadow as shadow


def fixture():
    bus = FakeBus()
    bus.state['position'][6] = 1751
    bus.state['position'][10] = 1753
    bus.state['position'][11] = 2300
    bus.state['goal'] = bus.state['position'].copy()
    return bus, {}, Clock()


class ResumeTests(unittest.TestCase):
    def test_empty_preview_starts_without_markers_even_between_fresh_frames(self):
        camera=object.__new__(held.Camera)
        camera.cv=Mock(); camera.title='test'; camera.report={}
        camera.require_policy_pose=False
        camera.next=Mock(side_effect=[(0.,np.zeros((2,2,3))),None])
        camera.detect=Mock(side_effect=AssertionError('Preparation must not detect cube markers'))
        camera.show=Mock(); camera.key=Mock(side_effect=[255,32])
        with patch.object(held.time,'monotonic',return_value=.01), \
             patch.object(held.time,'sleep'),patch.object(held.motion.base,'check_cancel'):
            camera.preview()
        camera.detect.assert_not_called()
        self.assertEqual(camera.report['preview_camera_gate'],dict(ready=True,markers_required=False))

    def test_empty_preview_still_requires_a_live_camera_frame(self):
        camera=object.__new__(held.Camera)
        camera.cv=Mock(); camera.title='test'; camera.report={}
        camera.require_policy_pose=False
        camera.next=Mock(return_value=None)
        camera.key=Mock(side_effect=[32,KeyboardInterrupt('cancel')])
        with patch.object(held.time,'monotonic',return_value=0.), \
             patch.object(held.time,'sleep'),patch.object(held.motion.base,'check_cancel'):
            with self.assertRaises(KeyboardInterrupt): camera.preview()
        self.assertEqual(camera.next.call_count,2)

    def test_preparation_camera_skips_calibration_and_tracker_setup(self):
        with patch('camera_stream.CameraStream') as stream, \
             patch.object(held.threading,'Thread'), \
             patch.object(held.Camera,'_setup_tracking',side_effect=AssertionError('No tracking for preparation')) as setup:
            stream.return_value.meta={}
            camera=held.Camera(dict(prepare_only=True))
        setup.assert_not_called()
        self.assertIsNone(camera.detector)

    def test_preview_saves_raw_pixels_on_unicode_path_even_between_frames(self):
        import cv2 as cv
        camera = object.__new__(held.Camera)
        camera.cv = SimpleNamespace(namedWindow=Mock(), resizeWindow=Mock(),
            WINDOW_NORMAL=cv.WINDOW_NORMAL, imencode=cv.imencode, error=cv.error)
        camera.title, camera.report = 'test', {}
        raw = np.arange(36, dtype=np.uint8).reshape(3,4,3)
        camera.next = Mock(side_effect=[(0., raw.copy()), None, None])
        camera.detect = Mock(return_value=None)
        camera.show = lambda frame, lines: frame.fill(255)
        camera.key = Mock(side_effect=[255, ord('s'), KeyboardInterrupt('finished')])
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)/'\ud55c\uae00 \uc800\uc7a5'
            folder.mkdir()
            with patch.object(shadow, 'ROOT', folder), \
                 patch.object(held.time, 'monotonic', return_value=0.), \
                 patch.object(held.time, 'sleep'), patch.object(held.motion.base, 'check_cancel'), \
                 contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(KeyboardInterrupt, 'finished'): camera.preview()
            saved = Path(camera.report['saved_vision_preview'])
            decoded = cv.imdecode(np.frombuffer(saved.read_bytes(), dtype=np.uint8), cv.IMREAD_COLOR)
            np.testing.assert_array_equal(decoded, raw)

    def test_preview_save_error_keeps_preview_running(self):
        import cv2 as cv
        for fault in (OSError('disk unavailable'), cv.error('encode failed'), ValueError('encode failed')):
            with self.subTest(fault=type(fault).__name__):
                camera = object.__new__(held.Camera)
                camera.cv = SimpleNamespace(namedWindow=Mock(), resizeWindow=Mock(),
                    WINDOW_NORMAL=cv.WINDOW_NORMAL, imencode=cv.imencode, error=cv.error)
                camera.title, camera.report = 'test', {}
                camera.next = Mock(return_value=(0., np.zeros((2,2,3), dtype=np.uint8)))
                camera.detect = Mock(return_value=None)
                camera.show = Mock()
                camera.key = Mock(side_effect=[ord('s'), KeyboardInterrupt('finished')])
                with patch.object(Path, 'write_bytes', side_effect=fault), \
                     patch.object(held.time, 'monotonic', return_value=0.), \
                     patch.object(held.time, 'sleep'), patch.object(held.motion.base, 'check_cancel'), \
                     contextlib.redirect_stdout(io.StringIO()):
                    with self.assertRaisesRegex(KeyboardInterrupt, 'finished'): camera.preview()
                self.assertEqual(camera.next.call_count, 2)
                self.assertEqual(camera.report['preview_save_errors'], [str(fault)])
                self.assertNotIn('saved_vision_preview', camera.report)

    def test_preview_waits_for_policy_quality_and_three_valid_frames(self):
        camera = object.__new__(held.Camera)
        camera.cv = Mock()
        camera.title = 'test'
        camera.report = {}
        camera.next = Mock(return_value=(0., np.zeros((1, 1))))
        good = dict(n_cube_markers=2, palm_minus_cube=[0,0,.01],
                    cube_quat_world=[1,0,0,0], reproj=1.)
        camera.detect = Mock(side_effect=[dict(good, reproj=4), good,
            dict(good, n_cube_markers=1), good, good, good])
        camera.key = Mock(return_value=32)
        camera.show = Mock()
        with patch.object(held.time, 'monotonic', return_value=0.), \
             patch.object(held.time, 'sleep'), patch.object(held.motion.base, 'check_cancel'):
            camera.preview()
        self.assertEqual(camera.detect.call_count, 6)
        self.assertEqual(camera.report['preview_policy_gate']['status'], 'ready')

    def test_policy_skip_reasons_preserve_quality_limits_and_history_reset(self):
        from test_encoder_policy_shadow import state, pose
        engine = shadow.Shadow(lambda obs: np.zeros(16))
        cases = [(None, 'no_pose'), (dict(pose(), n_cube_markers=1), 'cube_markers_below_2'),
                 (dict(pose(), palm_minus_cube=None), 'hand_cube_position_missing'),
                 (dict(pose(), cube_quat_world=None), 'cube_orientation_missing'),
                 (dict(pose(), reproj=4), 'reprojection_over_3px_or_invalid'),
                 (dict(pose(), cube_quat_world=[0,0,0,0]), 'quaternion_not_unit')]
        for bad, reason in cases:
            engine.step(state(), pose(), 50)
            self.assertIsNone(engine.step(state(), bad, 50))
            self.assertEqual(engine.last_skip_reason, reason)
            self.assertEqual(engine.count, 0)
        for _ in range(2): self.assertIsNone(engine.step(state(), pose(), 50))
        self.assertIsNotNone(engine.step(state(), pose(), 50))
        self.assertIsNone(engine.last_skip_reason)

    def test_failed_hold_records_vision_reason_and_turns_off(self):
        bus, report, clock = fixture()
        def callback(reader):
            engine = shadow.Shadow(lambda obs: np.zeros(16), expected_torque=1)
            held.fixed_hold(reader, Camera(clock, 'no_pose'), engine, report,
                            now=clock.now, sleep=clock.sleep, cancel=lambda: None)
        with self.assertRaisesRegex(RuntimeError, 'no_pose'):
            self.run_case(bus, report, clock, held_check=callback)
        self.assert_released(bus, report)
        self.assertEqual(report['held_policy_skip_counts']['no_pose'], report['held_camera_frames'])
        self.assertEqual(len(report['held_vision_diagnostics']), report['held_camera_frames'])

    def test_space_start_skips_typing_and_keeps_countdown_and_preflight(self):
        self.check_space_start('success')

    def test_policy_mode_routes_preparation_to_active_trial(self):
        self.check_space_start('policy_success')

    def test_contact_mode_routes_only_to_fixed_hold(self):
        self.check_space_start('contact_success')

    def test_loaded_policy_routes_to_active_trial_with_operator_input(self):
        self.check_space_start('loaded_success')

    def test_prepare_only_never_routes_to_policy_or_contact(self):
        self.check_space_start('prepare_success')

    def test_space_start_can_cancel_before_opening_motor_port(self):
        self.check_space_start('preview_cancel')

    def test_space_start_can_cancel_countdown_without_motion(self):
        self.check_space_start('countdown_cancel')

    def test_space_start_rejects_pose_change_during_countdown(self):
        self.check_space_start('pose_changed')

    def check_space_start(self, scenario):
        camera, bus = Mock(), Mock()
        initial = dict(position=np.full(16, 2048))
        fresh = dict(position=initial['position'].copy())
        if scenario == 'pose_changed': fresh['position'][0] += 2
        bus.preflight.side_effect = [initial, fresh]
        bus.block.return_value = dict(torque=np.zeros(16, dtype=int))
        runner = SimpleNamespace(__name__='fake_motion', Bus=Mock(return_value=bus),
            CAPS=motion.CAPS, plan=lambda raw: ([], raw, np.full(16, 2570)), run_trial=Mock())
        def preview():
            runner.Bus.assert_not_called()
            if scenario == 'preview_cancel': raise KeyboardInterrupt('preview cancelled')
        camera.preview.side_effect = preview
        if scenario == 'countdown_cancel': camera.key.side_effect = KeyboardInterrupt('countdown cancelled')
        with tempfile.TemporaryDirectory() as directory, contextlib.ExitStack() as stack:
            stack.enter_context(patch.object(held, 'Camera', return_value=camera))
            policy_factory=stack.enter_context(patch.object(shadow, 'Policy'))
            stack.enter_context(patch.object(shadow, 'Shadow'))
            stack.enter_context(patch.object(shadow, 'ROOT', Path(directory)))
            stack.enter_context(patch.object(held.motion.base, 'check_cancel'))
            sleep = stack.enter_context(patch.object(held.time, 'sleep'))
            typing = stack.enter_context(patch('builtins.input', side_effect=AssertionError('Unexpected typing')))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            result = held.main(motion_runner=runner, confirmation=None, result_stem='test',
                               policy_mode=scenario=='policy_success', contact_mode=scenario=='contact_success',
                               prepare_only=scenario=='prepare_success',loaded_policy_mode=scenario=='loaded_success')
            report = json.loads((Path(directory)/'test_result.json').read_text())
        typing.assert_not_called()
        camera.close.assert_called_once()
        self.assertTrue(camera.start_on_space)
        self.assertEqual(camera.require_policy_pose,scenario not in ('prepare_success','contact_success','loaded_success'))
        self.assertEqual(report['policy_actions_sent'], 0)
        if scenario in ('success','policy_success','contact_success','prepare_success','loaded_success'):
            self.assertEqual(result, 0)
            runner.run_trial.assert_called_once()
            self.assertEqual(bus.preflight.call_count, 2)
            self.assertAlmostEqual(sum(c.args[0] for c in sleep.call_args_list), 3.)
            self.assertEqual(report['start_trigger'], 'preview_space')
            if scenario=='policy_success':
                self.assertIn('active_check',runner.run_trial.call_args.kwargs)
                self.assertNotIn('held_check',runner.run_trial.call_args.kwargs)
                self.assertEqual(report['status'],'bounded_policy_motion_complete')
                self.assertTrue(report['policy_controls_motors'])
            if scenario=='loaded_success':
                self.assertIn('active_check',runner.run_trial.call_args.kwargs)
                self.assertNotIn('held_check',runner.run_trial.call_args.kwargs)
                self.assertEqual(report['status'],'loaded_policy_trial_complete')
                self.assertTrue(report['policy_controls_motors'])
            if scenario=='contact_success':
                self.assertIn('held_check',runner.run_trial.call_args.kwargs)
                self.assertNotIn('active_check',runner.run_trial.call_args.kwargs)
                self.assertEqual(report['status'],'contact_fixed_hold_complete')
                self.assertFalse(report['policy_controls_motors'])
            if scenario=='prepare_success':
                policy_factory.assert_not_called()
                self.assertIn('held_check',runner.run_trial.call_args.kwargs)
                self.assertNotIn('active_check',runner.run_trial.call_args.kwargs)
                self.assertEqual(report['status'],'empty_preparation_complete')
                self.assertFalse(report['policy_controls_motors'])
        else:
            self.assertEqual(result, 1)
            runner.run_trial.assert_not_called()
            if scenario == 'pose_changed': self.assertIn('Pose changed', report['error'])

    def test_preflight_distinguishes_no_response_device_error_and_wrong_model(self):
        for reply, message in (((0, -3001, 0), 'no response'),
                               ((0, -3002, 0), 'communication failed'),
                               ((1240, 0, 4), 'device error 4'),
                               ((1220, 0, 0), 'model=1220')):
            with self.subTest(reply=reply):
                bus = object.__new__(motion.Bus)
                bus.report = {}
                bus.port = None
                bus.sdk = SimpleNamespace(COMM_SUCCESS=0)
                bus.packet = SimpleNamespace(ping=lambda *args: reply)
                with self.assertRaisesRegex(RuntimeError, message): bus.preflight()
                self.assertEqual(bus.report['ping_events'], [dict(motor_id=1,
                    model_number=reply[0], communication=reply[1], device_error=reply[2])])

    def run_case(self, bus, report, clock, **kwargs):
        with contextlib.redirect_stdout(io.StringIO()):
            motion.run_trial(bus, bus.snapshot(), report, now=clock.now,
                             sleep=clock.sleep, cancel=lambda: None, **kwargs)

    def assert_released(self, bus, report):
        self.assertTrue(report['all_torque_off_verified'])
        self.assertTrue(report['settings_restored'])
        self.assertFalse(np.any(bus.state['torque']))
        torques = [v for a, v in bus.writes if a == 64]
        self.assertEqual(len(torques), 2)
        self.assertTrue(np.all(torques[0] == 1))
        self.assertTrue(np.all(torques[1] == 0))

    def test_posture_then_real_policy_inference_without_intermediate_off(self):
        bus, report, clock = fixture()
        before = []
        engine = shadow.Shadow(shadow.Policy(shadow.PROJECT/'policy/policy_np.npz'), expected_torque=1)
        def callback(reader):
            before.append(len(bus.writes))
            held.fixed_hold(reader, Camera(clock), engine, report,
                            now=clock.now, sleep=clock.sleep, cancel=lambda: None)
            self.assertEqual(before[0], len(bus.writes))
        self.run_case(bus, report, clock, held_check=callback)
        self.assert_released(bus, report)
        self.assertTrue(report['held_shadow_complete'])
        self.assertGreater(len(report['held_policy_samples']), 0)
        self.assertLess(clock.now(), 60)
        caps = [v for a, v in bus.writes if a == 102]
        np.testing.assert_array_equal(caps[0], motion.CAPS)
        self.assertEqual(len(caps), 2)  # one configuration, one restoration
        for sample in report['samples']:
            if sample['phase'] == 'prepare_other_15':
                self.assertEqual(sample['command_hw_raw'][11], 2300)
            if sample['phase'] == 'ID12_in_fixed_posture':
                np.testing.assert_array_equal(np.delete(sample['command_hw_raw'], 11),
                                              np.delete(report['goal_hw_raw'], 11))

    def install_lag(self, bus, report, clock, *, duration=.4, ticks=24):
        original = bus.snapshot
        waiting = []
        writes_during_wait = []
        def snapshot():
            state = original()
            if report.get('phase') == 'ID12_in_fixed_posture' and state['goal'][11] >= 2340:
                if not waiting:
                    waiting.append(clock.now())
                if clock.now()-waiting[0] < duration:
                    state['position'][11] -= ticks
                    writes_during_wait.append(len(bus.writes))
            return state
        bus.snapshot = snapshot
        return writes_during_wait

    def test_short_lag_pauses_same_target_then_continues(self):
        bus, report, clock = fixture()
        writes = self.install_lag(bus, report, clock)
        self.run_case(bus, report, clock)
        self.assert_released(bus, report)
        self.assertEqual(len(set(writes)), 1)
        events = report['tracking_wait_events']
        self.assertEqual([x['event'] for x in events], ['wait', 'resume'])
        self.assertLess(events[-1]['waited_s'], 2)
        goals = np.array([v for a, v in bus.writes if a == 116])
        self.assertLessEqual(np.max(np.abs(np.diff(goals, axis=0))), 2)

    def test_persistent_lag_releases_without_retry_or_more_force(self):
        bus, report, clock = fixture()
        self.install_lag(bus, report, clock, duration=100)
        with self.assertRaisesRegex(RuntimeError, 'within 2s'):
            self.run_case(bus, report, clock)
        self.assert_released(bus, report)
        self.assertEqual([x['event'] for x in report['tracking_wait_events']], ['wait'])
        self.assertEqual(report['failed_phase'], 'ID12_in_fixed_posture')

    def test_hard_faults_skip_soft_recovery_and_release(self):
        for fault in ('tracking', 'current', 'goal', 'temperature', 'read_timeout', 'held_drift'):
            with self.subTest(fault=fault):
                bus, report, clock = fixture()
                original = bus.snapshot
                def snapshot():
                    state = original()
                    if report.get('phase') == 'ID12_in_fixed_posture' and state['goal'][11] >= 2380:
                        if fault == 'tracking': state['position'][11] -= 50
                        if fault == 'current': state['current'][11] = 111
                        if fault == 'goal': state['goal'][11] += 1
                        if fault == 'temperature': state['temperature'][11] = 46
                        if fault == 'read_timeout': clock.sleep(.21)
                        if fault == 'held_drift': state['position'][0] -= 24
                    return state
                bus.snapshot = snapshot
                with self.assertRaises(RuntimeError): self.run_case(bus, report, clock)
                self.assert_released(bus, report)
                self.assertEqual(report['tracking_wait_events'], [])

    def test_camera_failure_and_callback_exception_release(self):
        for fault in ('lost', 'error', 'slow', 'cancel', 'no_pose', 'exception'):
            with self.subTest(fault=fault):
                bus, report, clock = fixture()
                def callback(reader):
                    if fault == 'exception': raise ValueError('inference failure')
                    engine = shadow.Shadow(lambda obs: np.zeros(16), expected_torque=1)
                    held.fixed_hold(reader, Camera(clock, fault), engine, report,
                                    now=clock.now, sleep=clock.sleep, cancel=lambda: None)
                with self.assertRaises((RuntimeError, KeyboardInterrupt, ValueError)):
                    self.run_case(bus, report, clock, held_check=callback)
                self.assert_released(bus, report)
                self.assertEqual(report['failed_phase'], 'fixed_start_pose_check')

    def test_cancel_partial_enable_and_failed_off(self):
        for fault in ('cancel', 'partial_enable', 'off_fails'):
            with self.subTest(fault=fault):
                bus, report, clock = fixture()
                if fault == 'partial_enable': bus.partial_enable = True
                if fault == 'off_fails':
                    bus.off_fails = True
                    self.install_lag(bus, report, clock, duration=100)
                def callback(reader):
                    if fault == 'cancel': raise KeyboardInterrupt()
                with self.assertRaises((RuntimeError, KeyboardInterrupt)):
                    self.run_case(bus, report, clock, held_check=callback)
                if fault == 'off_fails':
                    self.assertFalse(report['all_torque_off_verified'])
                    self.assertFalse(any(a == 102 and np.any(v == 2352) for a, v in bus.writes))
                else:
                    self.assertTrue(report['all_torque_off_verified'])

    def test_total_timeout_includes_shadow(self):
        bus, report, clock = fixture()
        def callback(reader):
            clock.sleep(61)
            reader()
        with self.assertRaisesRegex(RuntimeError, 'timeout'):
            self.run_case(bus, report, clock, held_check=callback)
        self.assert_released(bus, report)

    def test_invalid_start_and_latest_saved_id12_position_have_no_writes(self):
        # The most recent OFF position is historical, not a live measurement.
        saved = json.loads((ROOT/'hand_tip_settings_readonly_result.json').read_text())
        for raw in (saved['dynamic_state']['12']['present_raw'], 2048, 2570, 4096):
            with self.subTest(raw=raw):
                bus, report, clock = fixture()
                bus.state['position'][11] = raw
                bus.state['goal'] = bus.state['position'].copy()
                with self.assertRaises(ValueError): self.run_case(bus, report, clock)
                self.assertEqual(bus.writes, [])

    def test_wait_deadline_cannot_be_reset_by_late_good_sample(self):
        pace = motion.TrackingPace({}, 'test')
        self.assertFalse(pace.ready(.04, 1., 12))
        with self.assertRaisesRegex(RuntimeError, 'within 2s'):
            pace.ready(0., 3., 12)

    def test_recorded_id16_catchup_resumes_without_demanding_zero_error(self):
        # Actual paused target2179, positions2159->2167, t28.203->30.219.
        rows = [(28.203,2159), (28.344,2164), (28.484,2165),
                (28.641,2165), (28.766,2166), (29.328,2167), (30.219,2167)]
        report = {}
        pace = motion.TrackingPace(report, 'full_start_pose')
        results = [pace.ready((2179-position)*motion.TICK, t, 16) for t,position in rows]
        self.assertEqual(results, [False,False,False,False,True,True,True])
        self.assertEqual([x['event'] for x in report['tracking_wait_events']], ['wait','resume'])

    def test_loaded_residual_error_does_not_restart_waiting(self):
        pace = motion.TrackingPace({}, 'full_start_pose')
        self.assertFalse(pace.ready(.031, 0., 16))
        self.assertTrue(pace.ready(.019, .5, 16))
        for stamp in (1.,2.,3.):
            self.assertTrue(pace.ready(.019, stamp, 16))
        self.assertFalse(pace.ready(.031, 4., 16))
        with self.assertRaisesRegex(RuntimeError, 'within 2s'):
            pace.ready(.025, 6., 16)


if __name__ == '__main__': unittest.main()
