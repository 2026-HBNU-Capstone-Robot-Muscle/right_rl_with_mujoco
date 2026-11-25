"""Offline failure injection; no camera, serial port or motor SDK opened."""
import contextlib
import io
import unittest
from unittest.mock import patch
import numpy as np
import leap_thumb_prefix_empty as r

def fixture():
    a=r.p.source_position()
    return dict(position=a.copy(),goal=a.copy(),torque=np.zeros(16,dtype=int),error=np.zeros(16,dtype=int),
        watchdog=np.zeros(16,dtype=int),voltage=np.full(16,48),temperature=np.full(16,23),current=np.zeros(16,dtype=int),
        current_cap=np.full(16,550),acceleration=np.full(16,4),velocity=np.full(16,20),
        p_gain=np.full(16,2100),i_gain=np.zeros(16,dtype=int),d_gain=np.full(16,1900))

class FakeBus(r.Bus):
    def __init__(self,fault=None):
        self.state=fixture();self.events=[];self.goal_attempts=0;self.enabled=False
        self.off_reads=0;self.on_reads=0;self.fault=fault
    def snapshot(self):
        if self.goal_attempts==1 and not self.enabled:
            self.off_reads+=1
            if self.fault=='small_shift' and self.off_reads==1:
                self.state['goal'][0]-=1;self.state['position'][0]-=1
        if self.enabled and np.any(self.state['torque']):self.on_reads+=1
        s={k:v.copy() for k,v in self.state.items()}
        if self.fault=='prewrite_drift' and not self.events:s['position'][0]+=5
        if self.goal_attempts==1 and not self.enabled:
            if self.fault=='bad_profile':s['p_gain'][0]+=1
            if self.fault=='large_off_shift':s['goal'][0]+=5
            if self.fault=='unstable_off' and self.off_reads==2:s['goal'][0]+=2
        if self.enabled and np.any(s['torque']):
            if self.fault=='enable_wrap':s['position'][2]%=4096;s['position'][2]-=4096
            if self.fault=='unstable_enable' and self.on_reads==2:s['goal'][0]+=2
            if self.goal_attempts>1:
                if self.fault=='read_failure':raise RuntimeError('Injected read failure')
                if self.fault=='voltage':s['voltage'][0]=35
                if self.fault=='tracking':s['position'][0]-=120
                if self.fault=='fixed_joint':s['position'][4]+=21
                if self.fault=='goal_changed':s['goal'][0]+=1
                if self.fault=='current':s['current'][0]=600
        return s
    def transport(self,addr,size,values,verify=True):
        v=np.array(values,copy=True);self.events.append((addr,v))
        key={64:'torque',80:'d_gain',82:'i_gain',84:'p_gain',98:'watchdog',102:'current_cap',108:'acceleration',112:'velocity',116:'goal'}[addr]
        if addr==116:
            self.goal_attempts+=1
            if self.fault=='write_failure' and self.goal_attempts==3:raise RuntimeError('Injected write failure')
            self.state['position']=v.copy()
            if self.fault=='thumb_stall' and self.goal_attempts>=123:
                self.state['position'][2]=r.p.source_position()[2]
        if addr==64 and np.any(v):self.enabled=True
        self.state[key]=v
    def off(self):self.write(64,1,np.zeros(16,dtype=int))

class Tests(unittest.TestCase):
    def execute(self,bus,cancel=lambda:None,now=None):
        report={};kw={} if now is None else dict(now=now)
        with patch.object(r.checked.Bus,'write',FakeBus.transport),patch.object(r.checked,'check_cancel',lambda:None),contextlib.redirect_stdout(io.StringIO()):
            try:r.run(bus,{k:v.copy() for k,v in bus.state.items()},report,cancel=cancel,sleep=lambda _:None,**kw)
            except BaseException as exc:return report,exc
        return report,None

    def test_complete_plan_frozen_joint_goals_and_settings(self):
        b=FakeBus();report,exc=self.execute(b);self.assertIsNone(exc)
        self.assertEqual(report['status'],'empty_thumb_prefix_complete')
        self.assertTrue(report['all_torque_off_verified']);self.assertTrue(report['settings_restored'])
        goals=np.array([v for a,v in b.events if a==116]);base=r.p.source_position()
        self.assertEqual(len(goals),242)
        np.testing.assert_array_equal(goals[:,r.p.FROZEN],np.tile(base[r.p.FROZEN],(len(goals),1)))
        np.testing.assert_array_equal(goals[:122,:4],np.tile(base[:4],(122,1)))
        np.testing.assert_array_equal(goals[122:,r.p.ROOTS],np.full((120,3),2472))
        self.assertLessEqual(abs(np.diff(goals,axis=0)).max(),1)
        self.assertLessEqual(abs(goals[-1,:4]-base[:4]).max(),96)
        self.assertTrue(np.all(np.diff(goals[:,2])<=0));self.assertGreater(goals[-1,2],3900)
        for key in ('current_cap','acceleration','velocity','watchdog','p_gain','i_gain','d_gain'):
            np.testing.assert_array_equal(b.state[key],fixture()[key])
        self.assertFalse(any(a in (10,11,20,38) for a,_ in b.events))

    def test_source_hash_and_mapping_fail_closed(self):
        with patch.object(r.p,'SOURCE_SHA','wrong'):
            with self.assertRaises(ValueError):r.p.source_position()
        with patch.object(r.p,'SIM_TO_MOTOR_IDS',list(range(1,17))):
            with self.assertRaises(ValueError):r.p.source_position()

    def test_invalid_start_and_prewrite_drift_no_writes(self):
        for index,delta in ((0,33),(2,-4096)):
            b=FakeBus();b.state['position'][index]+=delta
            report,exc=self.execute(b);self.assertIsNotNone(exc);self.assertEqual(b.events,[])
        b=FakeBus('prewrite_drift');report,exc=self.execute(b)
        self.assertIsNotNone(exc);self.assertEqual(b.events,[])

    def test_preload_failures_do_not_enable(self):
        for fault in ('bad_profile','large_off_shift','unstable_off'):
            with self.subTest(fault=fault):
                b=FakeBus(fault);report,exc=self.execute(b)
                self.assertIsNotNone(exc);self.assertFalse(b.enabled)
                self.assertTrue(report['all_torque_off_verified']);self.assertTrue(report['settings_restored'])

    def test_live_faults_off_restore_without_goal_retry(self):
        for fault in ('enable_wrap','unstable_enable','read_failure','voltage','tracking','fixed_joint','goal_changed','current','write_failure'):
            with self.subTest(fault=fault):
                b=FakeBus(fault);report,exc=self.execute(b);self.assertIsNotNone(exc)
                self.assertTrue(report['all_torque_off_verified']);self.assertTrue(report['settings_restored'])
                self.assertEqual(sum(a==64 and np.any(v) for a,v in b.events),1)
                self.assertEqual(b.goal_attempts,1 if fault in ('enable_wrap','unstable_enable') else 3 if fault=='write_failure' else 2)

    def test_stationary_thumb_stops_before_full_prefix(self):
        b=FakeBus('thumb_stall');report,exc=self.execute(b)
        self.assertIn('no progress',str(exc));self.assertLess(b.goal_attempts,242)
        self.assertTrue(report['all_torque_off_verified']);self.assertTrue(report['settings_restored'])

    def test_camera_fault_or_user_cancel_stops_motion(self):
        for error in (RuntimeError('camera stale'),KeyboardInterrupt('user Q')):
            b=FakeBus()
            def cancel():
                if b.goal_attempts>=130:raise error
            report,exc=self.execute(b,cancel=cancel)
            self.assertIs(exc,error);self.assertTrue(report['all_torque_off_verified'])
            self.assertTrue(report['settings_restored']);self.assertEqual(b.goal_attempts,130)

    def test_clock_deadline_stops_before_path(self):
        b=FakeBus();times=iter([0.,36.])
        report,exc=self.execute(b,now=lambda:next(times))
        self.assertIn('35s',str(exc));self.assertEqual(b.goal_attempts,2)
        self.assertTrue(report['all_torque_off_verified'])

    def test_small_stable_shift_adopted_once(self):
        b=FakeBus('small_shift');report,exc=self.execute(b)
        self.assertIsNone(exc);self.assertEqual(report['enable_goal_shift_ticks'][0],-1)
        self.assertEqual(report['thumb_travel_ticks'][2],-96)
        with self.assertRaises(ValueError):b.adopt(report['accepted_enable_goal'])

    def test_transport_rejects_unscheduled_goal_and_turn_wrap(self):
        base=r.p.source_position()
        for index,delta in ((0,1),(2,-4096),(4,1),(7,1)):
            b=FakeBus();b.bind(base);q=base.copy();q[index]+=delta
            with patch.object(r.checked.Bus,'write',FakeBus.transport):
                with self.assertRaises(ValueError):b.write(116,4,q)
            self.assertEqual(b.events,[])

    def test_source_neighborhood_boundary(self):
        base=r.p.source_position()
        for i in range(16):
            for sign in (-1,1):
                q=base.copy();q[i]+=sign*32;rows=r.p.plan(q)
                self.assertLessEqual(abs(rows[-1][1][:4]-q[:4]).max(),96)
                q[i]+=sign
                with self.assertRaises(ValueError):r.p.plan(q)

if __name__=='__main__':unittest.main()
