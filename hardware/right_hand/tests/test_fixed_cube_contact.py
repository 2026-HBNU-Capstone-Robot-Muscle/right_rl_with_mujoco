import contextlib
import io
import unittest
from unittest.mock import Mock
import numpy as np
from test_hand_prepare_resume import fixture
import hand_prepare_resume as prep
import hand_prepare_contact as contact_prep
import encoder_policy_shadow as shadow
import fixed_cube_contact as contact


class Camera:
    def __init__(self, clock, report, scenario):
        self.clock, self.report, self.scenario = clock, report, scenario
        self.acknowledged = set()
        self.place_began = None
    def next(self, **kwargs):
        if self.scenario == 'camera_lost': return None
        return self.clock.now(), np.zeros((1,1,3),np.uint8)
    def detect(self, frame):
        if self.scenario == 'no_pose': return None
        return dict(n_cube_markers=2, palm_minus_cube=[.01,0,-.02],
                    cube_quat_world=[1,0,0,0], reproj=1.)
    def show(self, frame, lines): pass
    def key(self):
        phase = self.report.get('contact_phase')
        if self.scenario == 'cancel' and phase == 'observe': raise KeyboardInterrupt()
        if phase in ('place','remove') and phase not in self.acknowledged:
            if self.scenario=='late_place' and phase=='place':
                if self.place_began is None: self.place_began=self.clock.now()
                if self.clock.now()-self.place_began<self.report['contact_time_limits_s']['place']: return 255
            if self.scenario == 'no_place' and phase == 'place': return 255
            if self.scenario == 'no_remove' and phase == 'remove': return 255
            self.acknowledged.add(phase)
            return 32
        return 255


class Tests(unittest.TestCase):
    def run_case(self, scenario):
        bus, report, clock = fixture()
        engine = shadow.Shadow(lambda obs: np.zeros(16), expected_torque=1)
        camera = Camera(clock, report, scenario)
        writes_during = []
        routers = []
        def held(reader):
            before = len(bus.writes)
            if scenario == 'no_budget': clock.sleep(36-report['samples'][-1]['t'])
            if scenario == 'recorded_slow_prepare': clock.sleep(28.36-report['samples'][-1]['t'])
            if scenario == 'recorded_50_second_prepare': clock.sleep(50.532-report['samples'][-1]['t'])
            if scenario == 'overcurrent':
                normal = bus.snapshot
                def broken():
                    state = normal()
                    state['current'][7] = 91
                    return state
                bus.snapshot = broken
            try:
                keys = None
                if scenario == 'terminal_space':
                    class Console:
                        sent = set()
                        def kbhit(self):
                            p = report.get('contact_phase')
                            return p in ('place','remove') and p not in self.sent
                        def getwch(self):
                            self.sent.add(report['contact_phase'])
                            return ' '
                    camera.key = lambda:255
                    keys = contact.ContactKeys(camera, report, console=Console())
                    routers.append(keys)
                contact.run(reader, camera, engine, report, now=clock.now, sleep=clock.sleep,
                            cancel=keys.poll if keys else lambda:None, input_keys=keys)
            finally:
                writes_during.append(len(bus.writes)-before)
        error = None
        with contextlib.redirect_stdout(io.StringIO()):
            try:
                runner = contact_prep if scenario == 'recorded_50_second_prepare' else prep
                runner.run_trial(bus, bus.snapshot(), report, now=clock.now,
                               sleep=clock.sleep, cancel=lambda:routers[0].poll() if routers else None,
                               held_check=held)
            except (Exception,KeyboardInterrupt) as exc: error = exc
        self.assertEqual(writes_during, [0])
        self.assertTrue(report['all_torque_off_verified'])
        self.assertTrue(report['settings_restored'])
        self.assertFalse(np.any(bus.state['torque']))
        self.assertEqual(len([v for a,v in bus.writes if a==64]), 2)
        return report, error

    def test_success_real_observation_no_goal_writes_then_off(self):
        report, error = self.run_case('success')
        self.assertIsNone(error)
        self.assertTrue(report['contact_fixed_hold_complete'])
        self.assertTrue(report['cube_placement_confirmed'] and report['cube_removal_confirmed'])
        self.assertGreater(len(report['contact_inferences']), 3)
        self.assertEqual(report['contact_summary']['policy_actions_sent'], 0)
        np.testing.assert_allclose(report['contact_summary']['median_palm_minus_cube_m'], [.01,0,-.02])

    def test_terminal_space_survives_nested_health_reads(self):
        report, error = self.run_case('terminal_space')
        self.assertIsNone(error)
        self.assertTrue(report['contact_fixed_hold_complete'])
        self.assertEqual(report['contact_key_events'], [
            dict(phase='place',source='terminal',key='SPACE'),
            dict(phase='remove',source='terminal',key='SPACE')])

    def test_missing_pose_removes_cube_before_reporting_failure(self):
        report, error = self.run_case('no_pose')
        self.assertIn('No eligible', str(error))
        self.assertTrue(report['cube_removal_confirmed'])

    def test_placement_timeout_still_offers_removal(self):
        report, error = self.run_case('no_place')
        self.assertIn('placement was not confirmed', str(error))
        self.assertFalse(report['cube_placement_confirmed'])
        self.assertTrue(report['cube_removal_confirmed'])

    def test_removal_timeout_does_not_report_success(self):
        report, error = self.run_case('no_remove')
        self.assertIn('removal not confirmed', str(error))
        self.assertFalse(report['cube_removal_confirmed'])
        self.assertNotIn('contact_fixed_hold_complete', report)

    def test_no_time_budget_refuses_placement(self):
        report, error = self.run_case('no_budget')
        self.assertIn('Insufficient ON time', str(error))
        self.assertNotIn('contact_phase', report)

    def test_recorded_28_second_preparation_gets_16_second_placement(self):
        report, error = self.run_case('recorded_slow_prepare')
        self.assertIsNone(error)
        self.assertEqual(report['contact_time_limits_s']['place'], 16.)
        self.assertTrue(report['contact_fixed_hold_complete'])
        self.assertLess(report['samples'][-1]['t'], 60.)

    def test_budget_preserves_observation_removal_and_margin(self):
        for used, expected in [(0,20), (25,20), (28.36,16), (35,10)]:
            window = contact.placement_seconds(used)
            self.assertEqual(window, expected)
            self.assertLessEqual(used+window+5+8+2, 60.)
        for used in (35.01,60,None,float('nan'),float('inf'),-1):
            with self.assertRaises(RuntimeError): contact.placement_seconds(used)

    def test_recorded_50_second_preparation_completes_with_90_second_budget(self):
        report, error = self.run_case('recorded_50_second_prepare')
        self.assertIsNone(error)
        self.assertEqual(report['contact_time_limits_s']['place'],20.)
        self.assertTrue(report['contact_fixed_hold_complete'])
        self.assertLess(report['samples'][-1]['t'],90.)
        self.assertLessEqual(50.532+20+5+8+2,90.)
        for used, expected in [(50.532,20),(60,15),(65,10)]:
            self.assertEqual(contact.placement_seconds(used,total_on_seconds=90),expected)
        with self.assertRaises(RuntimeError): contact.placement_seconds(65.01,total_on_seconds=90)

    def test_late_space_does_not_start_observation_after_placement_deadline(self):
        report, error = self.run_case('late_place')
        self.assertIn('placement was not confirmed', str(error))
        self.assertFalse(report['cube_placement_confirmed'])
        self.assertTrue(report['cube_removal_confirmed'])

    def test_camera_cancel_and_current_failures_release(self):
        for scenario, message in [('camera_lost','No fresh camera'), ('cancel',''),
                                  ('overcurrent','above trial threshold')]:
            with self.subTest(scenario=scenario):
                report, error = self.run_case(scenario)
                self.assertIsNotNone(error)
                self.assertIn(message, str(error))
                self.assertNotIn('contact_fixed_hold_complete', report)


class KeyTests(unittest.TestCase):
    def fixture(self, chars='', camera_keys=None):
        class Console:
            buffer = list(chars)
            def kbhit(self): return bool(self.buffer)
            def getwch(self): return self.buffer.pop(0)
        report = dict(phase='fixed_start_pose_check',contact_phase='place')
        camera = Mock()
        camera.key.side_effect = camera_keys
        camera.key.return_value = 255
        return contact.ContactKeys(camera,report,Console()), report

    def test_space_from_either_window_is_consumed_once(self):
        for chars, keys in [(' ',None), ('',[32,255,255,255])]:
            router, report = self.fixture(chars,keys)
            router.poll(); router.poll()
            self.assertEqual(router.key(),32)
            self.assertEqual(router.key(),255)
            self.assertEqual(len(report['contact_key_events']),1)

    def test_q_esc_ctrl_c_are_not_hidden_by_queued_space(self):
        for key in ('q','Q','\x1b','\x03'):
            router,_ = self.fixture(' '+key)
            with self.assertRaises(KeyboardInterrupt): router.poll()
        router,_ = self.fixture(camera_keys=[ord('Q')])
        with self.assertRaises(KeyboardInterrupt): router.poll()

    def test_preparation_observation_and_old_phase_space_do_not_confirm(self):
        for phase,contact_phase in [('full_start_pose','place'),('fixed_start_pose_check','observe')]:
            router, report = self.fixture(' ')
            report.update(phase=phase,contact_phase=contact_phase)
            router.poll()
            report.update(phase='fixed_start_pose_check',contact_phase='remove')
            self.assertEqual(router.key(),255)
        router,report = self.fixture(' ')
        router.poll()
        report['contact_phase']='remove'
        self.assertEqual(router.key(),255)

    def test_extended_key_scan_code_is_not_treated_as_q(self):
        router,_ = self.fixture('\xe0q ')
        self.assertEqual(router.key(),32)


if __name__ == '__main__': unittest.main()
