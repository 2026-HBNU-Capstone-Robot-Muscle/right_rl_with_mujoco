import contextlib
import io
import unittest
import numpy as np
from test_hand_start_pose import FakeBus, Clock
import hand_prepare_resume as prep

# Recorded 2026-09-28 preflight. No motors are accessed by these tests.
RAW = np.array([2441,2651,2018,1880,2006,1964,1765,1907,
                2070,1969,1759,1903,2016,1851,1760,1907])


def fixture():
    bus, report, clock = FakeBus(), {}, Clock()
    bus.state['position'] = RAW.copy()
    bus.state['goal'] = RAW.copy()
    return bus, report, clock


class PreparationTests(unittest.TestCase):
    def test_80ma_profile_is_scoped_and_restored_after_run(self):
        np.testing.assert_array_equal(np.flatnonzero(prep.CAPS!=prep.runner.CAPS),[7,15])
        np.testing.assert_array_equal(prep.CAPS[[7,15]],[80,80])
        np.testing.assert_array_equal(prep.runner.CAPS[[7,15]],[60,60])
        bus,report,clock=fixture()
        initial=bus.state['current_cap'].copy()
        initial_velocity=bus.state['velocity'].copy()
        self.run_case(bus,report,clock)
        writes=[v for a,v in bus.writes if a==102]
        self.assertEqual(len(writes),2)
        np.testing.assert_array_equal(writes[0],prep.CAPS)
        np.testing.assert_array_equal(writes[1],initial)
        velocities=[v for a,v in bus.writes if a==112]
        np.testing.assert_array_equal(velocities[0],np.full(16,20))
        np.testing.assert_array_equal(velocities[1],initial_velocity)
        for s in report['samples']:
            np.testing.assert_array_equal(s['state_all_ids']['current_cap'],prep.CAPS)
            np.testing.assert_array_equal(s['state_all_ids']['velocity'],np.full(16,20))

    def test_fast_steps_reduce_in_stages_without_accelerating_again(self):
        pace=prep.runner.AdaptiveSteps({},'full_start_pose',caps=prep.CAPS,max_step_ticks=8)
        error=np.zeros(16); current=np.zeros(16)
        self.assertEqual(pace.update(error,current,0.)[15],8)
        error[15]=.011
        self.assertEqual(pace.update(error,current,1.)[15],4)
        error[15]=.021
        self.assertEqual(pace.update(error,current,2.)[15],2)
        error[15]=0
        steps=pace.update(error,current,3.)
        self.assertEqual(steps[15],2)
        self.assertTrue(np.all(steps[:15]==8))

    def test_80ma_profile_keeps_90ma_measured_stop_for_ids8_and16(self):
        for index in (7,15):
            bus,report,clock=fixture()
            normal=bus.snapshot
            def overcurrent():
                state=normal()
                if report.get('phase')=='full_start_pose': state['current'][index]=91
                return state
            bus.snapshot=overcurrent
            with self.assertRaisesRegex(RuntimeError,'above trial threshold 90'):
                self.run_case(bus,report,clock)
            self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
            self.assertEqual(len([v for a,v in bus.writes if a==64]),2)

    def test_adaptive_steps_use_selected_current_profile(self):
        pace=prep.runner.AdaptiveSteps({},'full_start_pose',caps=prep.CAPS)
        error=np.zeros(16); error[15]=.012
        current=np.zeros(16,dtype=int); current[15]=60
        self.assertEqual(pace.update(error,current,0.)[15],4)
        current[15]=73
        self.assertEqual(pace.update(error,current,1.)[15],2)

    def test_preflight_checks_device_current_limit_against_80ma(self):
        from types import SimpleNamespace
        for index in (7,15):
            bus=object.__new__(prep.Bus)
            bus.report={}; bus.port=None
            bus.sdk=SimpleNamespace(COMM_SUCCESS=0)
            bus.packet=SimpleNamespace(ping=lambda *a:(1240,0,0))
            def read(motor,addr,size=1):
                if addr==38: return 79 if motor==index+1 else 2352
                return {64:0,70:0,10:0,11:5,98:0,6:38,20:0}[addr]
            bus.read=read
            with self.assertRaisesRegex(RuntimeError,f'ID {index+1}: current limit below trial cap'):
                bus.preflight()

    def test_recorded_id16_load_triggers_slowdown_before_wait_band(self):
        report={}
        pace=prep.runner.AdaptiveSteps(report,'full_start_pose')
        for command,actual,current in ((2478,2470,33),(2486,2476,29),(2494,2482,57)):
            error=np.zeros(16); error[15]=(command-actual)*prep.TICK
            currents=np.zeros(16,dtype=int); currents[15]=current
            steps=pace.update(error,currents,0.)
        self.assertEqual(steps[15],2)
        self.assertTrue(np.all(steps[:15]==4))
        event=report['adaptive_step_events'][0]
        self.assertLess(event['error_rad'],.035)
        steps=pace.update(np.zeros(16),np.zeros(16),1.)
        self.assertEqual(steps[15],2)  # Do not repeatedly accelerate a loaded joint.
        self.assertEqual(len(report['adaptive_step_events']),1)

    def test_signed_current_and_tracking_both_slow_only_affected_joints(self):
        pace=prep.runner.AdaptiveSteps({},'full_start_pose')
        error=np.zeros(16); error[0]=.021; error[1]=.012
        current=np.zeros(16,dtype=int); current[1]=65536-57
        steps=pace.update(error,current,0.)
        np.testing.assert_array_equal(steps[:2],[2,2])
        self.assertTrue(np.all(steps[2:]==4))

    def test_rate_limited_joint_continues_to_policy_callback_at_lower_speed(self):
        bus,report,clock=fixture()
        normal=bus.snapshot
        position=float(RAW[15]); previous=clock.now()
        def limited_servo():
            nonlocal position,previous
            state=normal()
            dt=clock.now()-previous; previous=clock.now()
            if np.any(state['torque']):
                position+=np.clip(state['goal'][15]-position,-40*dt,40*dt)
                state['position'][15]=int(round(position))
                if abs(state['goal'][15]-position)>=10: state['current'][15]=57
            return state
        bus.snapshot=limited_servo
        entered=[]
        self.run_case(bus,report,clock,active_check=lambda control:entered.append(control.read()))
        self.assertEqual(len(entered),1)
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
        self.assertEqual(report['tracking_wait_events'],[])
        self.assertEqual(report['adaptive_step_events'][0]['motor_id'],16)
        samples=[s for s in report['samples'] if s['phase']=='full_start_pose']
        commands=np.array([s['command_hw_raw'] for s in samples])
        self.assertEqual(np.max(np.abs(np.diff(commands[:,0]))),8)
        self.assertLess(clock.now(),60)
        self.assertEqual(len([v for a,v in bus.writes if a==64]),2)
        np.testing.assert_array_equal([v for a,v in bus.writes if a==102][0],prep.CAPS)

    def install_preload_drift(self, bus, report, offsets, *, mismatch=False):
        normal=bus.snapshot
        calls=[]
        original=bus.state['position'].copy()
        def snapshot():
            if report.get('phase')=='configure_preload_OFF':
                offset=offsets[min(len(calls),len(offsets)-1)]
                calls.append(offset)
                bus.state['position'][13]=original[13]+offset
                bus.state['goal'][13]=original[13]+offset+(2 if mismatch else 0)
            return normal()
        bus.snapshot=snapshot
        return calls

    def test_two_tick_off_drift_settles_before_single_enable(self):
        bus, report, clock=fixture()
        self.install_preload_drift(bus,report,[0,-2,-2])
        self.run_case(bus,report,clock)
        self.assertEqual(report['preload_shift_ticks'][13],-2)
        self.assertEqual(len(report['preload_samples']),3)
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
        self.assertEqual(len([v for a,v in bus.writes if a==64 and np.any(v)]),1)
        on=next(i for i,(a,v) in enumerate(bus.writes) if a==64 and np.any(v))
        self.assertEqual(len([v for a,v in bus.writes[:on] if a==116]),1)

    def test_off_drift_never_accumulates_or_ignores_goal_position_mismatch(self):
        for offsets,mismatch in (([0,3,6],False),([0],True),([-3,3]*20,False)):
            bus,report,clock=fixture()
            self.install_preload_drift(bus,report,offsets,mismatch=mismatch)
            with self.assertRaises(RuntimeError): self.run_case(bus,report,clock)
            self.assertFalse(any(a==64 and np.any(v) for a,v in bus.writes))
            self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
            self.assertLess(clock.now(),.6)

    def test_off_settling_still_checks_health_slow_reads_and_cancel(self):
        for fault in ('temperature','current_cap','profile','slow','cancel'):
            bus,report,clock=fixture()
            normal=bus.snapshot
            def snapshot():
                state=normal()
                if report.get('phase')=='configure_preload_OFF':
                    if fault=='temperature': state['temperature'][0]=46
                    if fault=='current_cap': state['current_cap'][0]=1000
                    if fault=='profile': state['velocity'][0]=21
                    if fault=='slow': clock.sleep(.21)
                    if fault=='cancel': raise KeyboardInterrupt()
                return state
            bus.snapshot=snapshot
            with self.assertRaises((RuntimeError,KeyboardInterrupt)): self.run_case(bus,report,clock)
            self.assertFalse(any(a==64 and np.any(v) for a,v in bus.writes))
            self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])

    def test_full_start_allows_existing_final_tolerance_during_motion(self):
        bus, report, clock = fixture()
        original = bus.snapshot
        def loaded():
            state = original()
            if report.get('phase') == 'full_start_pose' and state['goal'][15]-RAW[15] >= 40:
                state['position'][15] -= 20  # .03068rad: inside existing .035 acceptance.
            return state
        bus.snapshot = loaded
        self.run_case(bus, report, clock)
        self.assertTrue(report['all_torque_off_verified'])
        self.assertEqual(report['status'], 'start_pose_reached_before_torque_off')
        self.assertEqual(report['tracking_wait_events'], [])
        self.assertEqual(report['tracking_pace_by_phase']['full_start_pose']['resume_rad'], .035)

    def test_full_start_stall_outside_final_tolerance_still_stops(self):
        bus, report, clock = fixture()
        original = bus.snapshot
        def stalled():
            state = original()
            if report.get('phase') == 'full_start_pose': state['position'][15] = RAW[15]
            return state
        bus.snapshot = stalled
        with self.assertRaisesRegex(RuntimeError, 'within 2s'):
            self.run_case(bus, report, clock)
        self.assertEqual(report['failed_phase'], 'full_start_pose')
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
        self.assertEqual(len([v for a,v in bus.writes if a == 64]), 2)

    def test_both_recorded_id16_plateaus_are_within_full_start_tolerance(self):
        pace = prep.runner.TrackingPace({}, 'full_start_pose', settled_error=.035)
        # First/latest logs: paused errors20ticks, settling residuals18/17ticks.
        for t, ticks in ((0,20),(.2,18),(1,18),(2.1,18),(3,20),(3.2,17),(5.1,17)):
            self.assertTrue(pace.ready(ticks*prep.TICK, t, 16))
        self.assertFalse(pace.ready(.05, 6., 16))
        self.assertTrue(pace.ready(.0260776733940559, 6.2, 16))
        with self.assertRaises(ValueError):
            prep.runner.TrackingPace({}, 'bad', settled_error=.10)

    def run_case(self, bus, report, clock, **kwargs):
        with contextlib.redirect_stdout(io.StringIO()):
            prep.run_trial(bus, bus.snapshot(), report, cancel=lambda: None,
                           now=clock.now, sleep=clock.sleep, **kwargs)

    def test_recorded_pose_plan_recovers_three_pips_preserves_policy_limits(self):
        lower = prep.base.limits.SAFE_LOWERS.copy()
        moves, staged, goal = prep.plan(RAW)
        self.assertEqual([i+1 for i, target in moves], [7,11,15])
        for i, target in moves:
            self.assertGreater(target, RAW[i])
            self.assertLessEqual((target-RAW[i])*prep.TICK, .15)
        np.testing.assert_array_equal(prep.base.target_plan(staged), goal)
        np.testing.assert_array_equal(lower, prep.base.limits.SAFE_LOWERS)
        self.assertLessEqual(np.max(np.abs(goal-staged))*prep.TICK, 1.25)
        self.assertLessEqual(np.max(np.abs(goal-RAW))*prep.TICK, 1.40)

    def test_recorded_pose_completes_with_read_delay_and_continuous_hold(self):
        bus, report, clock = fixture()
        original = bus.snapshot
        def delayed():
            clock.sleep(.032)
            return original()
        bus.snapshot = delayed
        def held(reader):
            began = clock.now()
            while clock.now()-began < 10:
                reader()
                clock.sleep(.01)
        self.run_case(bus, report, clock, held_check=held)
        self.assertEqual(report['status'], 'start_pose_reached_before_torque_off')
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
        self.assertLess(clock.now(), 60)
        on_off = [v for a,v in bus.writes if a == 64]
        self.assertEqual(len(on_off), 2)
        self.assertTrue(np.all(on_off[0] == 1) and np.all(on_off[1] == 0))
        currents = [v for a,v in bus.writes if a == 102]
        np.testing.assert_array_equal(currents[0], prep.CAPS)
        goals = np.array([v for a,v in bus.writes if a == 116])
        self.assertEqual(np.max(np.abs(np.diff(goals,axis=0))), 8)
        for phase in ('recover_ID7', 'recover_ID11', 'recover_ID15', 'full_start_pose'):
            commands = np.array([s['command_hw_raw'] for s in report['samples'] if s['phase'] == phase])
            self.assertEqual(np.max(np.abs(np.diff(commands, axis=0))),
                             8 if phase == 'full_start_pose' else 2)
        self.assertTrue(any(s['phase']=='recover_ID15' for s in report['samples']))
        self.assertFalse(any(s['phase']=='ID12_in_fixed_posture' for s in report['samples']))

    def test_invalid_pose_has_no_writes(self):
        for index,raw in ((14,1700),(14,1722),(11,1750),(0,4096)):
            bus, report, clock = fixture()
            bus.state['position'][index] = raw
            bus.state['goal'] = bus.state['position'].copy()
            with self.assertRaises(ValueError): self.run_case(bus, report, clock)
            self.assertEqual(bus.writes, [])

    def test_low_allowed_pip_uses_two_bounded_segments(self):
        for raw_value in (1723,1725,1730,1734):
            raw = RAW.copy()
            raw[14] = raw_value
            moves, staged, goal = prep.plan(raw)
            segments = [target for i,target in moves if i==14]
            self.assertEqual(len(segments), 2)
            deltas = np.diff([raw_value]+segments)
            self.assertTrue(np.all(deltas>0))
            self.assertTrue(np.all((deltas+4)*prep.TICK<=.15))
            self.assertLessEqual((segments[-1]-raw_value+4)*prep.TICK,.21)
            np.testing.assert_array_equal(prep.base.target_plan(staged),goal)

    def test_recorded_id15_1730_recovers_without_reenable_or_current_increase(self):
        bus,report,clock = fixture()
        bus.state['position'][14] = 1730
        bus.state['goal'] = bus.state['position'].copy()
        self.run_case(bus,report,clock)
        self.assertEqual(len([m for m in report['recovery_moves'] if m[0]==14]),2)
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
        self.assertEqual(len([v for a,v in bus.writes if a==64]),2)
        caps = [v for a,v in bus.writes if a==102]
        self.assertEqual(len(caps),2)
        self.assertEqual(caps[0][14],60)
        goals = np.array([v for a,v in bus.writes if a==116])
        self.assertLessEqual(np.max(np.abs(np.diff(goals,axis=0))),8)

    def test_id15_stall_turns_off_without_current_increase_or_reenable(self):
        bus, report, clock = fixture()
        original = bus.snapshot
        def stalled():
            state = original()
            if report.get('phase') == 'recover_ID15': state['position'][14] = RAW[14]
            return state
        bus.snapshot = stalled
        with self.assertRaisesRegex(RuntimeError, 'within 2s'):
            self.run_case(bus, report, clock)
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
        self.assertEqual(report['failed_phase'], 'recover_ID15')
        self.assertEqual(len([v for a,v in bus.writes if a == 64]), 2)

    def test_old_id12_diagnostic_still_rejects_large_move(self):
        raw = RAW.copy()
        raw[[6,10,14]] = 1853
        with self.assertRaises(ValueError): prep.runner.plan(raw)
        prep.plan(raw)


if __name__ == '__main__': unittest.main()
