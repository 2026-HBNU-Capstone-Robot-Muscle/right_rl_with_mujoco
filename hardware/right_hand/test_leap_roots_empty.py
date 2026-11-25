"""Offline failure injection. Never imports a hardware SDK or opens a port."""
import contextlib
import io
import json
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import leap_roots_empty as r


def fixture():
    a = r.source_position()
    return dict(position=a.copy(), goal=a.copy(), torque=np.zeros(16,dtype=int),
                error=np.zeros(16,dtype=int), watchdog=np.zeros(16,dtype=int),
                voltage=np.full(16,48), temperature=np.full(16,23), current=np.zeros(16,dtype=int),
                current_cap=np.full(16,550), acceleration=np.full(16,4), velocity=np.full(16,20),
                p_gain=np.full(16,2100), i_gain=np.zeros(16,dtype=int), d_gain=np.full(16,1900))


class FakeBus(r.Bus):
    def __init__(self, *, fault=None):
        self.state=fixture()
        self.events=[]
        self.fault=fault
        self.goal_attempts=0
        self.enabled=False
        self.off_snapshots=0
        self.on_snapshots=0

    def snapshot(self):
        if self.goal_attempts==1 and not self.enabled:
            self.off_snapshots+=1
            if self.fault=='small_off_shift' and self.off_snapshots==1:
                self.state['goal'][[0,7]]-=1
                self.state['position'][[0,7]]-=1
        if self.enabled and np.any(self.state['torque']):
            self.on_snapshots+=1
            if self.fault=='small_enable_shift' and self.on_snapshots==1:
                self.state['goal'][[0,11]]-=1
                self.state['position'][[0,11]]-=1
        result={k:v.copy() for k,v in self.state.items()}
        if self.fault=='off_drift' and not self.events:
            result['position'][0] += 5
        if self.fault=='bad_preload' and self.events and not self.enabled:
            result['goal'][2] = 2
        if self.goal_attempts==1 and not self.enabled:
            if self.fault=='bad_profile':result['p_gain'][0]+=1
            if self.fault=='off_five_ticks':result['goal'][0]+=5
            if self.fault=='off_unstable' and self.off_snapshots==2:result['goal'][0]+=2
        if self.enabled and np.any(result['torque']):
            if self.fault=='enable_wrap':
                result['position'][2] = 2
            if self.fault=='enable_unstable' and self.on_snapshots==2:result['goal'][0]+=2
            if self.goal_attempts > 1:
                if self.fault=='ongoal_changed':result['goal'][0]+=1
                if self.fault=='stale_read':
                    raise RuntimeError('Injected communication failure')
                if self.fault=='low_voltage':
                    result['voltage'][0]=35
                if self.fault=='tracking':
                    result['position'][7]-=120
                if self.fault=='held_joint':
                    result['position'][2]+=21
        return result

    def transport(self, addr, size, values, verify=True):
        values=np.array(values,copy=True)
        self.events.append((addr,values))
        key={64:'torque',80:'d_gain',82:'i_gain',84:'p_gain',98:'watchdog',
             102:'current_cap',108:'acceleration',112:'velocity',116:'goal'}[addr]
        if addr==116:
            self.goal_attempts+=1
            if self.fault=='goal_write' and self.goal_attempts==2:
                raise RuntimeError('Injected goal write failure')
            self.state['position']=values.copy()
        if addr==64 and np.any(values):
            self.enabled=True
        self.state[key]=values

    def off(self):
        self.write(64,1,np.zeros(16,dtype=int))


class RootTests(unittest.TestCase):
    def execute(self,bus,initial=None):
        report={}
        with patch.object(r.checked.Bus,'write',FakeBus.transport), \
             patch.object(r.checked,'check_cancel',lambda:None), \
             contextlib.redirect_stdout(io.StringIO()):
            try:
                r.run(bus,fixture() if initial is None else initial,report,cancel=lambda:None,sleep=lambda _:None)
            except BaseException as exc:
                return report,exc
        return report,None

    def test_success_exact_multiturn_frozen_goals_and_restoration(self):
        bus=FakeBus()
        report,exc=self.execute(bus)
        self.assertIsNone(exc)
        self.assertEqual(report['status'],'empty_root_lift_complete')
        self.assertTrue(report['all_torque_off_verified'])
        self.assertTrue(report['settings_restored'])
        goals=np.array([v for a,v in bus.events if a==116])
        np.testing.assert_array_equal(goals[:,2],r.source_position()[2])
        np.testing.assert_array_equal(goals[:,r.FROZEN],np.tile(r.source_position()[r.FROZEN],(len(goals),1)))
        np.testing.assert_array_equal(goals[-1,r.ROOT_INDICES],[2472]*3)
        self.assertLessEqual(np.diff(goals,axis=0).max(),5)
        self.assertTrue(np.all(np.diff(goals[:,r.ROOT_INDICES],axis=0)>=0))
        for key in ('current_cap','acceleration','velocity','watchdog','p_gain','i_gain','d_gain'):
            np.testing.assert_array_equal(bus.state[key],fixture()[key])
        self.assertTrue(np.all(bus.state['torque']==0))
        self.assertFalse(any(a in (10,11,20,38) for a,_ in bus.events))

    def test_stale_source_pose_never_written(self):
        initial=fixture();initial['position'][2]=2
        bus=FakeBus()
        with self.assertRaises(ValueError):
            r.run(bus,initial,{},cancel=lambda:None,sleep=lambda _:None)
        self.assertEqual(bus.events,[])

    def test_prewrite_drift_no_writes(self):
        bus=FakeBus(fault='off_drift')
        report,exc=self.execute(bus)
        self.assertIsNotNone(exc)
        self.assertEqual(bus.events,[])

    def test_bad_preload_never_enables(self):
        bus=FakeBus(fault='bad_preload')
        report,exc=self.execute(bus)
        self.assertIsNotNone(exc)
        self.assertFalse(any(a==64 and np.any(v) for a,v in bus.events))
        self.assertTrue(report['all_torque_off_verified'])

    def test_failures_stop_off_and_restore_without_goal_retry(self):
        for fault in ('enable_wrap','stale_read','low_voltage','tracking','held_joint','goal_write','ongoal_changed'):
            with self.subTest(fault=fault):
                bus=FakeBus(fault=fault)
                report,exc=self.execute(bus)
                self.assertIsNotNone(exc)
                self.assertTrue(report['all_torque_off_verified'])
                self.assertTrue(report['settings_restored'])
                self.assertEqual(bus.goal_attempts,1 if fault=='enable_wrap' else 2)
                self.assertEqual(sum(a==64 and np.any(v) for a,v in bus.events),1)

    def test_small_stable_shift_adopted_once_then_frozen(self):
        for fault in ('small_off_shift','small_enable_shift'):
            with self.subTest(fault=fault):
                bus=FakeBus(fault=fault);report,exc=self.execute(bus)
                self.assertIsNone(exc)
                self.assertEqual(report['enable_goal_shift_ticks'][0],-1)
                self.assertEqual(len(report['off_preload_snapshots']),2)
                self.assertEqual(len(report['enable_snapshots']),2)
                goals=np.array([v for a,v in bus.events if a==116])
                frozen=np.array(report['accepted_enable_goal'])[r.FROZEN]
                np.testing.assert_array_equal(goals[1:,r.FROZEN],np.tile(frozen,(len(goals)-1,1)))
                self.assertEqual(len(goals),r.STEPS+2)
                with self.assertRaises(ValueError):bus.adopt_enable_hold(report['accepted_enable_goal'])

    def test_profile_and_large_or_unstable_off_shift_still_block_enable(self):
        for fault in ('bad_profile','off_five_ticks','off_unstable'):
            with self.subTest(fault=fault):
                bus=FakeBus(fault=fault);report,exc=self.execute(bus)
                self.assertIsNotNone(exc)
                self.assertFalse(bus.enabled)
                self.assertTrue(report['settings_restored'])
                self.assertGreaterEqual(len(report['off_preload_snapshots']),1)
                if fault=='bad_profile':self.assertIn('p_gain ID1',str(exc))

    def test_unstable_enable_stops_without_hold_or_path_write(self):
        bus=FakeBus(fault='enable_unstable');report,exc=self.execute(bus)
        self.assertIsNotNone(exc)
        self.assertEqual(bus.goal_attempts,1)
        self.assertTrue(report['all_torque_off_verified'])

    def test_guard_rejects_frozen_joint_change_and_jump_before_transport(self):
        for index,delta in ((2,-4096),(0,1),(7,6),(7,-1)):
            bus=FakeBus();start=r.source_position();bus.bind(start)
            goal=start.copy();goal[index]+=delta
            with patch.object(r.checked.Bus,'write',FakeBus.transport):
                with self.assertRaises(ValueError):bus.write(116,4,goal)
            self.assertEqual(bus.events,[])

    def test_source_and_mapping_cannot_be_silently_replaced(self):
        with patch.object(r,'SOURCE_SHA','wrong'):
            with self.assertRaises(ValueError):r.source_position()
        with patch.object(r.mapping,'SIM_TO_MOTOR_IDS',r.mapping.LEGACY_SIM_TO_MOTOR_IDS):
            with self.assertRaises(ValueError):r.source_position()

    def test_shared_single_turn_validator_stays_strict(self):
        multi=r.source_position();multi[2]=4098
        with self.assertRaises(ValueError):r.checked.validate_raw(multi)

    def test_original_4098_source_still_preserved_without_wrapping(self):
        with patch.object(r,'SOURCE',r.ROOT/'right_hand_start_20261019/source_readonly.json'), \
             patch.object(r,'SOURCE_SHA','faa0c6227b8f14dc68a3cd8d928837f6f72709c7bb303b8a34e67b53917b74d8'):
            bus=FakeBus();report,exc=self.execute(bus)
            self.assertIsNone(exc)
            self.assertEqual(report['id3_held_raw'],4098)
            self.assertTrue(all(v[2]==4098 for a,v in bus.events if a==116))

    def test_current_source_and_failure_message(self):
        self.assertEqual(r.source_position()[7],2402)
        bad=r.source_position();bad[15]+=33
        with self.assertRaisesRegex(ValueError,r'ID16 2411->2444.*\+33 ticks'):
            r.validate_start(bad)

    def test_real_preflight_rejects_mode3_before_any_write(self):
        bus=FakeBus()
        bus.report={};bus.sdk=SimpleNamespace(COMM_SUCCESS=0);bus.port=None
        bus.packet=SimpleNamespace(ping=lambda port,motor:(1240,0,0))
        values={64:0,70:0,10:0,11:5,98:0,6:53,20:0,38:2352}
        bus.read=lambda motor,addr,size=1:3 if motor==3 and addr==11 else values[addr]
        with self.assertRaisesRegex(RuntimeError,'register 11'):bus.preflight()
        self.assertEqual(bus.events,[])

    def test_real_preflight_accepts_source_mode5_without_wrapping(self):
        bus=FakeBus()
        bus.report={};bus.sdk=SimpleNamespace(COMM_SUCCESS=0);bus.port=None
        bus.packet=SimpleNamespace(ping=lambda port,motor:(1240,0,0))
        values={64:0,70:0,10:0,11:5,98:0,6:53,20:0,38:2352}
        bus.read=lambda motor,addr,size=1:values[addr]
        self.assertEqual(bus.preflight()['position'][2],r.source_position()[2])
        self.assertEqual(bus.events,[])

    def test_each_reviewed_pose_runs_and_records_its_own_source(self):
        entries=r.reviewed_starts()
        self.assertEqual(len(entries),3)
        for entry in entries:
            with self.subTest(source=entry['path']):
                bus=FakeBus();initial=fixture()
                initial['position']=entry['position'].copy()
                initial['goal']=entry['position'].copy()
                bus.state={k:v.copy() for k,v in initial.items()}
                report,exc=self.execute(bus,initial)
                self.assertIsNone(exc)
                self.assertEqual(report['source_sha256'],entry['sha256'])
                held=np.array([v for a,v in bus.events if a==116])[:,r.FROZEN]
                np.testing.assert_array_equal(held,np.tile(initial['position'][r.FROZEN],(len(held),1)))

    def test_mixed_joint_values_from_different_reviewed_poses_rejected(self):
        entries=r.reviewed_starts()
        morning=next(e for e in entries if e['position'][2]==4098)
        latest=next(e for e in entries if e['position'][1]==2056)
        mixed=morning['position'].copy();mixed[1]=latest['position'][1]
        with self.assertRaisesRegex(ValueError,'all 3 reviewed starts'):
            r.validate_start(mixed)

    def test_bad_additional_reference_hash_fails_closed(self):
        extra=r.ADDITIONAL_REVIEWED_SOURCES[0]
        with patch.object(r,'ADDITIONAL_REVIEWED_SOURCES',((extra[0],'incorrect'),)):
            with self.assertRaisesRegex(ValueError,'Reviewed source changed'):
                r.validate_start(r.source_position())

    def test_latest_22_23_27_tick_drift_uses_measured_start_not_reference(self):
        record=json.loads((r.ROOT/'right_hand_root_drift_20261020/source_preflight.json').read_text())
        actual=np.array(record['preflight_state']['position'])
        initial=fixture();initial['position']=actual.copy();initial['goal']=actual.copy()
        bus=FakeBus();bus.state={k:v.copy() for k,v in initial.items()}
        report,exc=self.execute(bus,initial)
        self.assertIsNone(exc)
        self.assertEqual([report['start_difference_ticks'][i] for i in (1,9,13)],[22,23,27])
        goals=np.array([v for a,v in bus.events if a==116])
        np.testing.assert_array_equal(goals[0],actual)
        np.testing.assert_array_equal(goals[:,r.FROZEN],np.tile(actual[r.FROZEN],(len(goals),1)))

    def test_start_boundary_does_not_expand_enable_shift(self):
        a=r.source_position();a[0]+=32
        np.testing.assert_array_equal(r.validate_start(a),a)
        a[0]+=1
        with self.assertRaises(ValueError):r.validate_start(a)
        bus=FakeBus();base=r.source_position();bus.bind(base)
        bad=base.copy();bad[0]+=5
        with self.assertRaises(ValueError):bus.adopt_enable_hold(bad)
        self.assertEqual(bus.events,[])


if __name__=='__main__': unittest.main()
