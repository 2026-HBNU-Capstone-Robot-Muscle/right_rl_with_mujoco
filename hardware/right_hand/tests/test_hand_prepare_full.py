"""Offline full-profile integration: no serial/camera devices are opened."""
import contextlib
import io
import unittest
from types import SimpleNamespace
import numpy as np
from test_hand_prepare_resume import fixture
from test_fixed_cube_contact import Camera
import hand_prepare_full as full
import encoder_policy_shadow as shadow
import fixed_cube_contact as contact


def full_fixture():
    bus,report,clock=fixture()
    for name,value in [('p_gain',2100),('i_gain',0),('d_gain',1900)]:
        bus.state[name]=np.full(16,value)
    normal=bus.write
    def write(addr,size,values,verify=True):
        if addr in (80,82,84):
            bus.writes.append((addr,np.array(values).copy()))
            bus.state[{80:'d_gain',82:'i_gain',84:'p_gain'}[addr]]=np.array(values).copy()
        else: normal(addr,size,values,verify)
    bus.write=write
    return bus,report,clock


class Tests(unittest.TestCase):
    def run_trial(self,bus,report,clock,**kwargs):
        with contextlib.redirect_stdout(io.StringIO()):
            full.run_trial(bus,bus.snapshot(),report,now=clock.now,sleep=clock.sleep,cancel=lambda:None,**kwargs)

    def released(self,bus,report):
        self.assertTrue(report['all_torque_off_verified'] and report['settings_restored'])
        self.assertFalse(np.any(bus.state['torque']))
        for name,value in [('p_gain',2100),('i_gain',0),('d_gain',1900),('current_cap',2352),('velocity',0)]:
            np.testing.assert_array_equal(bus.state[name],np.full(16,value))

    def test_entrypoints_share_full_profile_and_preserve_device_gains(self):
        import leap_prepare,leap_resume,leap_contact
        for entry in (leap_prepare,leap_resume,leap_contact): self.assertIs(entry.motion,full)
        self.assertTrue(full.PRESERVE_DEVICE_GAINS)
        self.assertTrue(np.all(full.CAPS==550) and np.all(full.STOPS==650))

    def test_preparation_reaches_target_faster_and_restores_after_off(self):
        bus,report,clock=full_fixture()
        self.run_trial(bus,report,clock)
        self.released(bus,report)
        self.assertLess(clock.now(),20)
        self.assertIsNone(report['total_on_limit_s'])
        full_rows=[s for s in report['samples'] if s['phase']=='full_start_pose']
        steps=np.abs(np.diff([s['command_hw_raw'] for s in full_rows],axis=0))
        self.assertEqual(int(np.max(steps)),16)
        for s in report['samples']:
            np.testing.assert_array_equal(s['state_all_ids']['current_cap'],full.CAPS)
            for name,value in [('p_gain',2100),('i_gain',0),('d_gain',1900)]:
                np.testing.assert_array_equal(s['state_all_ids'][name],np.full(16,value))
        off=next(i for i,(a,v) in enumerate(bus.writes) if a==64 and not np.any(v))
        self.assertTrue(any(i>off for i,(a,v) in enumerate(bus.writes) if a==84 and np.all(v==2100)))
        self.assertEqual(report['gain_source'],'preserved_preflight_per_motor')

    def test_per_motor_gains_are_preserved_without_remapping_or_scaling(self):
        bus,report,clock=full_fixture()
        bus.state['p_gain'][5]=1575
        bus.state['d_gain'][5]=1425
        initial=bus.snapshot()
        self.run_trial(bus,report,clock)
        for name in ('p_gain','i_gain','d_gain'):
            np.testing.assert_array_equal(report['position_gains_by_hw_id'][name],initial[name])
            np.testing.assert_array_equal(bus.state[name],initial[name])
            for s in report['samples']:
                np.testing.assert_array_equal(s['state_all_ids'][name],initial[name])

    def test_slowdown_recovers_only_after_stable_tracking(self):
        pace=full.RecoveringSteps({},'test')
        zero=np.zeros(16)
        for t in (0.,.41,.82): pace.update(zero,zero,t)
        self.assertTrue(np.all(pace.steps==16))
        self.assertTrue(np.all(pace.update(np.full(16,.045),zero,1.)==2))
        pace.update(zero,zero,1.1)
        self.assertTrue(np.all(pace.update(zero,zero,1.3)==2))
        self.assertTrue(np.all(pace.update(zero,zero,1.51)==4))
        pace.update(zero,zero,1.92); pace.update(zero,zero,2.33)
        self.assertTrue(np.all(pace.steps==16))
        # Stable error alone cannot speed up a saturated motor; signed currents count.
        self.assertTrue(np.all(pace.update(np.full(16,.025),np.full(16,65536-550),3.)==4))
        self.assertTrue(np.all(pace.update(zero,np.full(16,550),4.)==4))

    def test_healthy_hold_has_no_combined_90_second_deadline(self):
        bus,report,clock=full_fixture()
        def held(reader):
            clock.sleep(121); reader()
        self.run_trial(bus,report,clock,held_check=held)
        self.assertGreater(report['samples'][-1]['t'],120)
        self.released(bus,report)

    def test_contact_waits_for_late_placement_and_removal(self):
        bus,report,clock=full_fixture()
        engine=shadow.Shadow(lambda obs:np.zeros(16),expected_torque=1)
        class OperatorCamera(Camera):
            def key(self):
                phase=report.get('contact_phase')
                if phase in ('place','remove') and phase not in self.acknowledged:
                    self.acknowledged.add(phase)
                    clock.sleep(31 if phase=='place' else 12)
                    return 32
                return 255
        camera=OperatorCamera(clock,report,'success')
        def held(reader):
            clock.sleep(70)
            contact.run(reader,camera,engine,report,now=clock.now,sleep=clock.sleep)
        self.run_trial(bus,report,clock,held_check=held)
        self.assertTrue(report['contact_fixed_hold_complete'])
        self.assertGreater(report['samples'][-1]['t'],90)
        self.assertIsNone(report['contact_time_limits_s']['place'])
        self.assertIsNone(report['contact_time_limits_s']['remove'])
        self.released(bus,report)

    def test_faults_and_cancel_during_operator_wait_still_release(self):
        for fault in ('current','negative_current','temperature','gain','cancel','read_timeout','stuck'):
            with self.subTest(fault=fault):
                bus,report,clock=full_fixture()
                def held(reader):
                    if fault=='cancel': raise KeyboardInterrupt()
                    normal=bus.snapshot
                    def snapshot():
                        s=normal()
                        if fault=='current': s['current'][7]=651
                        if fault=='negative_current': s['current'][7]=65536-651
                        if fault=='temperature': s['temperature'][0]=46
                        if fault=='gain': s['p_gain'][0]=0
                        if fault=='read_timeout': clock.sleep(.21)
                        if fault=='stuck': s['position'][0]-=80
                        return s
                    bus.snapshot=snapshot
                    reader()
                with self.assertRaises((RuntimeError,KeyboardInterrupt)):
                    self.run_trial(bus,report,clock,held_check=held)
                self.released(bus,report)

    def test_preparation_stall_releases_without_current_escalation(self):
        bus,report,clock=full_fixture()
        normal=bus.snapshot
        frozen=[None]
        def snapshot():
            s=normal()
            if report.get('phase')=='full_start_pose':
                if frozen[0] is None: frozen[0]=s['position'].copy()
                s['position']=frozen[0].copy()
            return s
        bus.snapshot=snapshot
        with self.assertRaisesRegex(RuntimeError,'tracking did not recover|no preparation progress'):
            self.run_trial(bus,report,clock)
        self.released(bus,report)
        self.assertEqual(len([v for a,v in bus.writes if a==102]),2)

    def test_gain_write_failure_and_partial_enable_release(self):
        for fault in ('gain_write','partial_enable'):
            bus,report,clock=full_fixture()
            if fault=='gain_write':
                normal=bus.write
                failed=[False]
                def write(addr,size,values,verify=True):
                    normal(addr,size,values,verify)
                    if addr==84 and not failed[0]:
                        failed[0]=True
                        raise RuntimeError('gain readback failed')
                bus.write=write
            else: bus.partial_enable=True
            with self.assertRaises(RuntimeError): self.run_trial(bus,report,clock)
            self.released(bus,report)

    def test_missing_gain_snapshot_refuses_before_writes(self):
        bus,report,clock=fixture()
        with self.assertRaises(ValueError): self.run_trial(bus,report,clock)
        self.assertEqual(bus.writes,[])

    def test_unverified_off_does_not_restore_gains_or_current(self):
        bus,report,clock=full_fixture()
        bus.off_fails=True
        with self.assertRaisesRegex(RuntimeError,'CUT MOTOR POWER'):
            self.run_trial(bus,report,clock)
        self.assertFalse(report['all_torque_off_verified'])
        self.assertNotIn('settings_restored',report)
        for address in (80,82,84,102):
            self.assertEqual(len([v for a,v in bus.writes if a==address]),1)

    def test_current_limit_preflight_requires_550(self):
        for limit in (549,550):
            fake,_,_=full_fixture()
            bus=object.__new__(full.Bus)
            bus.report={}; bus.port=None; bus.sdk=SimpleNamespace(COMM_SUCCESS=0)
            bus.packet=SimpleNamespace(ping=lambda *args:(1240,0,0))
            bus.read=lambda motor,addr,size=1:limit if addr==38 else {64:0,70:0,10:0,11:5,98:0,6:38,20:0}[addr]
            bus.snapshot=fake.snapshot
            if limit==549:
                with self.assertRaisesRegex(RuntimeError,'current limit below'): bus.preflight()
            else: bus.preflight()

    def test_policy_uses_training_period_and_bounded_32_tick_steps(self):
        bus,report,clock=full_fixture()
        def active(control):
            control.deadline=clock.now()+10
            clock.sleep(.051)
            action=control.apply(np.ones(16),clock.now())
            self.assertIsNotNone(action)
            self.assertEqual(np.max(np.abs(control.command-control.anchor)),32)
            before=len(bus.writes)
            control.apply(-np.ones(16),clock.now())
            self.assertEqual(before,len(bus.writes))
            clock.sleep(.051)
            control.apply(-np.ones(16),clock.now())
        self.run_trial(bus,report,clock,active_check=active)
        self.released(bus,report)
        self.assertEqual(report['policy_control_period_s'],.05)
        self.assertEqual(report['policy_action_scale'],.2)
        self.assertEqual(report['policy_step_ticks'],32)


if __name__=='__main__': unittest.main()
