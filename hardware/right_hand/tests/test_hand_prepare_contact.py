import contextlib
import io
import unittest
from types import SimpleNamespace
import numpy as np
from test_hand_prepare_resume import fixture
import hand_prepare_resume as resume
import hand_prepare_contact as contact


class Tests(unittest.TestCase):
    def test_profile_changes_only_loaded_joints_in_contact_entrypoint(self):
        # Historical 150mA profile remains available for replay/regression tests.
        np.testing.assert_array_equal(np.flatnonzero(contact.CAPS!=resume.CAPS), [2,7,11,15])
        np.testing.assert_array_equal(np.flatnonzero(contact.STOPS!=resume.STOPS), [2,7,11,15])
        for motor in contact.LOADED_IDS:
            self.assertEqual((contact.CAPS[motor-1],contact.STOPS[motor-1]),(150,180))
        self.assertEqual((resume.CAPS[7],resume.STOPS[7]),(80,90))
        np.testing.assert_array_equal(contact.CAPS[[6,10,14]], [60,60,60])
        self.assertEqual((contact.CAPS[12],contact.STOPS[12]),(100,110))

    def trial(self, fault=None, timeout_phase=None):
        bus, report, clock = fixture()
        initial = bus.snapshot()
        normal = bus.snapshot
        def snapshot():
            s = normal()
            if np.all(s['torque']==1):
                for motor in contact.LOADED_IDS: s['current'][motor-1] = 150
                if fault is not None and report.get('phase')=='fixed_start_pose_check':
                    motor, current = fault
                    s['current'][motor-1] = current % 65536
            return s
        bus.snapshot = snapshot
        def held(reader):
            before = len(bus.writes)
            if timeout_phase == 'hold': clock.sleep(91)
            reader()
            self.assertEqual(len(bus.writes), before)
        def cancel():
            if timeout_phase == 'prepare' and report.get('phase') == 'full_start_pose':
                clock.sleep(61)
        error = None
        with contextlib.redirect_stdout(io.StringIO()):
            try:
                contact.run_trial(bus, initial, report, now=clock.now,
                    sleep=clock.sleep,cancel=cancel,held_check=held)
            except RuntimeError as exc: error=exc
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
        self.assertEqual(len([v for a,v in bus.writes if a==64]),2)
        caps = [v for a,v in bus.writes if a==102]
        self.assertEqual(len(caps),2)
        np.testing.assert_array_equal(caps[0],contact.CAPS)
        np.testing.assert_array_equal(caps[1],initial['current_cap'])
        np.testing.assert_array_equal(report['measured_current_stop_ma_by_hw_id'],contact.STOPS)
        self.assertEqual(report['total_on_limit_s'],90)
        self.assertEqual(report['preparation_limit_s'],60)
        return error

    def test_150ma_passes_preload_enable_and_fixed_hold_then_restores(self):
        self.assertIsNone(self.trial())

    def test_preparation_still_stops_at_60_seconds(self):
        self.assertIn('Preparation exceeded 60-second limit',str(self.trial(timeout_phase='prepare')))

    def test_fixed_hold_stops_at_absolute_90_seconds(self):
        self.assertIn('90-second total torque-on timeout',str(self.trial(timeout_phase='hold')))

    def test_extended_limit_rejects_active_policy_before_writes(self):
        bus, report, _ = fixture()
        with self.assertRaisesRegex(ValueError,'fixed hold only'):
            resume.runner.run_trial(bus,bus.snapshot(),report,full_start=True,
                held_check=lambda reader:None,active_check=lambda control:None,total_on_seconds=90)
        self.assertEqual(bus.writes,[])

    def test_signed_overcurrent_during_contact_releases(self):
        for motor in contact.LOADED_IDS:
            for current in (181,-181):
                with self.subTest(motor=motor,current=current):
                    self.assertIn(f'ID {motor} measured input current',str(self.trial((motor,current))))

    def test_preflight_requires_current_limit_150(self):
        for motor_id,limit in [(m,149) for m in contact.LOADED_IDS]+[(8,150)]:
            fake,_,_=fixture()
            bus=object.__new__(contact.Bus)
            bus.report={}; bus.port=None
            bus.sdk=SimpleNamespace(COMM_SUCCESS=0)
            bus.packet=SimpleNamespace(ping=lambda *a:(1240,0,0))
            def read(motor,addr,size=1):
                if addr==38: return limit if motor==motor_id else 2352
                return {64:0,70:0,10:0,11:5,98:0,6:38,20:0}[addr]
            bus.read=read; bus.snapshot=fake.snapshot
            if limit==149:
                with self.assertRaisesRegex(RuntimeError,f'ID {motor_id}: current limit below trial cap'): bus.preflight()
            else: bus.preflight()


if __name__ == '__main__': unittest.main()
