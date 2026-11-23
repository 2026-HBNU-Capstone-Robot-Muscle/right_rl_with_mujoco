"""Device-free checks of loaded adoption, replay and failure cleanup."""
import copy, time, unittest
from unittest.mock import patch
import numpy as np
import leap_measured_grip_hold_20261117 as pose


def state(torque):
    _, actual, goal = pose.reference()
    s = {name:np.zeros(16,dtype=int) for name,_,_ in pose.FIELDS}
    s.update(torque=np.full(16,torque), position=actual.copy(), goal=goal.copy(),
        p_gain=np.full(16,600), d_gain=np.full(16,200), current_cap=np.full(16,450),
        voltage=np.full(16,50), temperature=np.full(16,25))
    if not torque: s['goal']=actual.copy()
    return s


class Clock:
    def __init__(self):self.t=0.
    def now(self):return self.t
    def sleep(self,seconds):self.t+=seconds


class FakeBus:
    def __init__(self,initial):
        self.state=copy.deepcopy(initial)
        self.report={}
        self.writes=[]
        self.plan_module=type('Plan',(),{'NEW_PAIR_GRASP_MODE':True})()
        self.cancel_callback=lambda:None
        self.reads=0
        self.fault_after=None
        self.last_goal_command=self.previous_goal_command=None
    def snapshot(self):
        self.reads+=1
        s=copy.deepcopy(self.state)
        if self.fault_after is not None and self.reads>=self.fault_after and np.any(s['torque']):
            s['current'][3]=600
        return s
    def write(self,address,size,values):
        values=np.asarray(values).copy()
        self.writes.append((address,values))
        key={64:'torque',98:'watchdog',102:'current_cap',108:'acceleration',112:'velocity',116:'goal'}[address]
        self.state[key]=values.copy()
        if address==116:
            self.previous_goal_command=self.last_goal_command
            self.last_goal_command=values.copy()
            if np.any(self.state['torque']):self.state['position']=values.copy()
    def off(self):
        self.write(64,1,np.zeros(16,dtype=int))
        if np.any(self.snapshot()['torque']):raise RuntimeError('OFF not confirmed')
        self.report['all_torque_off_verified']=True


class PoseTests(unittest.TestCase):
    def run_fake(self,bus,initial,rows,limit=1.):
        clock=Clock()
        report=bus.report
        report['status']='started'
        start=initial['goal'].copy() if np.all(initial['torque']) else initial['position'].copy()
        pose.run_pose(bus,initial,start,rows,report,lambda:None,
            now=clock.now,sleep=clock.sleep,limit=limit)
        return report

    def test_exact_loaded_goal_not_measured_angle(self):
        initial=state(1)
        target=pose.reference()[2]
        self.assertEqual(initial['position'][11],2098)
        self.assertEqual(target[11],2124)
        bus=FakeBus(initial)
        report=self.run_fake(bus,initial,pose.path(initial['goal'],target))
        goals=[v for a,v in bus.writes if a==116]
        self.assertTrue(goals)
        self.assertTrue(all(np.array_equal(v,target) for v in goals))
        self.assertTrue(report['holding'])
        off=[n for n,(a,v) in enumerate(bus.writes) if a==64 and not np.any(v)]
        self.assertTrue(all(n>max(i for i,(a,_) in enumerate(bus.writes) if a==116) for n in off))
        self.assertTrue(report['all_torque_off_verified'])
        self.assertTrue(report['settings_restored'])

    def test_off_preloads_actual_then_moves_to_loaded_target(self):
        initial=state(0)
        target=pose.reference()[2]
        bus=FakeBus(initial)
        rows=pose.path(initial['position'],target)
        report=self.run_fake(bus,initial,rows,limit=2.)
        goals=np.asarray([v for a,v in bus.writes if a==116])
        self.assertTrue(np.array_equal(goals[0],initial['position']))
        self.assertTrue(np.array_equal(goals[-1],target))
        self.assertLessEqual(np.max(abs(np.diff(goals,axis=0))),2)
        self.assertTrue(report['holding'])

    def test_health_fault_does_not_advance_then_turns_off(self):
        initial=state(0)
        bus=FakeBus(initial)
        bus.fault_after=4
        with self.assertRaisesRegex(RuntimeError,'measured current'):
            self.run_fake(bus,initial,pose.path(initial['position'],pose.reference()[2]))
        self.assertEqual(len([v for a,v in bus.writes if a==116]),1) # OFF preload only
        self.assertTrue(bus.report['all_torque_off_verified'])
        self.assertTrue(bus.report['settings_restored'])

    def test_only_recorded_id11_upper_is_scoped(self):
        from loaded_quarter_reentry_20261025 import limits
        _,native_high=limits()
        _,high,_=pose.bounds()
        changed=np.flatnonzero(high>np.floor(native_high))
        self.assertEqual(changed.tolist(),[10])
        self.assertEqual(high[10],3284)
        target=pose.reference()[2].copy()
        target[10]=3285
        with self.assertRaises(RuntimeError):pose.path(pose.reference()[1],target)

    def test_long_empty_start_rate_and_exact_endpoint(self):
        low,high,_=pose.bounds()
        start=np.clip(np.full(16,2048),low,high)
        target=pose.reference()[2]
        rows=pose.path(start,target)
        self.assertGreater(len(rows),500)
        self.assertLessEqual(np.max(abs(np.diff(np.vstack([start,rows]),axis=0))),2)
        self.assertTrue(np.array_equal(rows[-1],target))

    def test_eeprom_writes_rejected(self):
        bus=object.__new__(pose.PoseBus)
        for address in (8,11,20,38,48,52,80,82,84):
            with self.assertRaisesRegex(RuntimeError,'allowlist'):
                bus.write(address,1,np.zeros(16,dtype=int))

    def test_read_recovery_requires_two_clean_whole_states(self):
        bus=object.__new__(pose.PoseBus)
        bus.report={}
        bus.port=type('Port',(),{'getBytesAvailable':lambda self:0})()
        clean=state(1)
        with patch('grip_health_read_20261118.block_once',side_effect=[
            (None,-3002,0,{}),(clean,0,0,{}),(clean,0,0,{})]) as read:
            self.assertIs(bus.snapshot(),clean)
        self.assertEqual(read.call_count,3)
        self.assertTrue(bus.report['read_failures'][0]['recovered'])

    def test_existing_read_deadline_is_not_extended(self):
        bus=object.__new__(pose.PoseBus)
        bus.report={}
        deadline=time.perf_counter()-.01
        bus._horizontal_sample_deadline=deadline
        with patch('grip_health_read_20261118.block_once') as read:
            with self.assertRaisesRegex(RuntimeError,'200ms'):bus.snapshot()
        read.assert_not_called()
        self.assertEqual(bus._horizontal_sample_deadline,deadline)

    def test_two_corrupt_reads_need_two_fresh_complete_states_before_return(self):
        bus=object.__new__(pose.PoseBus);bus.report={}
        bus.port=type('Port',(),{'getBytesAvailable':lambda self:0})()
        clean=state(1);first=copy.deepcopy(clean);first['position'][0]+=1
        with patch('grip_health_read_20261118.block_once',side_effect=[
            (None,-3002,0,{'first_incomplete_id':2}),
            (None,-3002,0,{'first_incomplete_id':2}),
            (first,0,0,{}),(clean,0,0,{})]) as read:
            self.assertIs(bus.snapshot(),clean)
        self.assertEqual(read.call_count,4)
        self.assertTrue(all(v['recovered'] for v in bus.report['read_failures']))
        self.assertTrue(all(v['consecutive_clean_reads']==2 for v in bus.report['read_failures']))
        self.assertEqual(len(bus._read_recovery_times),2)
        self.assertNotIn('writes',bus.report)

    def test_three_bad_reads_still_stop_without_accepting_partial_state(self):
        bus=object.__new__(pose.PoseBus);bus.report={}
        bus.port=type('Port',(),{'getBytesAvailable':lambda self:0})()
        with patch('grip_health_read_20261118.block_once',return_value=(None,-3002,0,{})) as read:
            with self.assertRaisesRegex(RuntimeError,'Repeated/error'):bus.snapshot()
        self.assertEqual(read.call_count,3)
        self.assertTrue(all(not v['recovered'] for v in bus.report['read_failures']))

    def test_device_error_is_never_retried(self):
        bus=object.__new__(pose.PoseBus);bus.report={}
        with patch('grip_health_read_20261118.block_once',return_value=(None,0,128,{})) as read:
            with self.assertRaisesRegex(RuntimeError,'Repeated/error'):bus.snapshot()
        self.assertEqual(read.call_count,1)

    def test_recovery_rate_limit_remains_three_in_ten_seconds(self):
        bus=object.__new__(pose.PoseBus);bus.report={}
        bus._read_recovery_times=[time.perf_counter()]*3
        with patch('grip_health_read_20261118.block_once',return_value=(None,-3002,0,{})) as read:
            with self.assertRaisesRegex(RuntimeError,'3retries/10s'):bus.snapshot()
        self.assertEqual(read.call_count,1)

    def test_partial_configuration_failure_still_attempts_off(self):
        initial=state(1)
        bus=FakeBus(initial)
        original=bus.write
        def write(address,size,values):
            if address==108 and not bus.report.get('all_torque_off_verified'):
                raise RuntimeError('Injected configuration failure')
            return original(address,size,values)
        bus.write=write
        with self.assertRaisesRegex(RuntimeError,'configuration failure'):
            self.run_fake(bus,initial,pose.path(initial['goal'],pose.reference()[2]))
        self.assertTrue(bus.report['all_torque_off_verified'])
        self.assertFalse(any(a==116 for a,v in bus.writes))

    def test_off_verification_failure_attempts_all_individual_off(self):
        bus=object.__new__(pose.PoseBus)
        bus.report={}
        bus.port=object()
        bus.write=lambda *args:None
        s=state(1)
        off=state(0)
        states=iter([s,off])
        bus.snapshot=lambda:next(states)
        calls=[]
        class Packet:
            def write1ByteTxRx(self,port,motor,address,value):
                calls.append((motor,address,value))
                return (0,0)
        bus.packet=Packet()
        pose.PoseBus.off(bus)
        self.assertEqual(calls,[(i,64,0) for i in range(1,17)])
        self.assertTrue(bus.report['all_torque_off_verified'])

    def test_one_tick_off_hold_readback_is_adopted_without_rewrite(self):
        initial=state(0)
        bus=FakeBus(initial)
        original=bus.write
        def write(address,size,values):
            original(address,size,values)
            if address==116 and not np.any(bus.state['torque']):
                bus.state['goal'][7]+=1
                bus.state['position'][7]+=1
        bus.write=write
        result=self.run_fake(bus,initial,pose.path(initial['position'],pose.reference()[2]),limit=2.)
        self.assertTrue(result['off_preload_confirmation']['accepted'])
        self.assertEqual(result['off_preload_confirmation']['shift_from_fixed_anchor'][7],1)
        self.assertEqual(result['accepted_start_goal'][7],initial['position'][7]+1)
        goals=[v for a,v in bus.writes if a==116]
        moving=np.vstack([result['accepted_start_goal'],goals[1:]])
        self.assertLessEqual(np.max(abs(np.diff(moving,axis=0))),2)
        self.assertTrue(result['holding'])

    def test_off_drift_after_configuration_captured_before_first_goal(self):
        initial=state(0)
        bus=FakeBus(initial)
        original=bus.write
        def write(address,size,values):
            original(address,size,values)
            if address==112 and not bus.report.get('all_torque_off_verified'):
                bus.state['position'][7]+=12
        bus.write=write
        result=self.run_fake(bus,initial,pose.path(initial['position'],pose.reference()[2]),limit=2.)
        first=next(v for a,v in bus.writes if a==116)
        self.assertEqual(first[7],initial['position'][7]+12)
        self.assertEqual(result['fresh_off_start']['position'][7],first[7])
        self.assertTrue(result['holding'])

    def test_off_postwrite_shift_above_four_ticks_still_rejected(self):
        initial=state(0)
        bus=FakeBus(initial)
        original=bus.write
        def write(address,size,values):
            original(address,size,values)
            if address==116 and not np.any(bus.state['torque']):
                bus.state['position'][7]+=5
                bus.state['goal'][7]+=5
        bus.write=write
        with self.assertRaisesRegex(RuntimeError,'limit=4ticks'):
            self.run_fake(bus,initial,pose.path(initial['position'],pose.reference()[2]))
        self.assertFalse(any(a==64 and np.any(v) for a,v in bus.writes))
        self.assertTrue(bus.report['all_torque_off_verified'])

    def test_one_tick_enable_shift_is_adopted_but_moving_goals_stay_exact(self):
        initial=state(0)
        bus=FakeBus(initial)
        original=bus.write
        def write(address,size,values):
            original(address,size,values)
            if address==64 and np.any(values):
                bus.state['position'][7]+=1
                bus.state['goal'][7]+=1
        bus.write=write
        result=self.run_fake(bus,initial,pose.path(initial['position'],pose.reference()[2]),limit=2.)
        self.assertEqual(result['accepted_start_goal'][7],initial['position'][7]+1)
        self.assertTrue(result['holding'])

    def test_reported_id11_3301_returns_before_other_joints_move(self):
        start=pose.reference()[1].copy();start[10]=3301
        target=pose.reference()[2]
        rows=pose.path(start,target)
        self.assertEqual(rows[8,10],3284)
        others=np.arange(16)!=10
        self.assertTrue(np.all(rows[:9,others]==start[others]))
        self.assertTrue(np.all(np.diff(np.r_[start[10],rows[:,10]])<=0))
        self.assertTrue(np.array_equal(rows[-1],target))
        self.assertLessEqual(np.max(abs(np.diff(np.vstack([start,rows]),axis=0))),2)

    def test_outside_start_can_execute_return_without_expanding_endpoint(self):
        initial=state(0)
        initial['position'][10]=initial['goal'][10]=3301
        bus=FakeBus(initial)
        result=self.run_fake(bus,initial,pose.path(initial['position'],pose.reference()[2]),limit=3.)
        goals=np.asarray([v for a,v in bus.writes if a==116])
        self.assertTrue(result['holding'])
        self.assertEqual(int(np.max(goals[:,10])),3301)
        self.assertEqual(int(goals[-1,10]),3284)
        self.assertTrue(result['all_torque_off_verified'])

    def test_other_joints_can_return_from_outside_the_old_start_bounds(self):
        _,high,low=pose.bounds()
        target=pose.reference()[2]
        start=target.copy()
        start[3]=int(np.ceil(low[3]))-12
        start[15]=int(high[15])+12
        rows=pose.path(start,target)
        whole=np.vstack([start,rows])
        excess=np.maximum(np.maximum(low-whole,whole-high),0.)
        self.assertTrue(np.all(np.diff(excess,axis=0)<=0))
        self.assertTrue(np.array_equal(rows[-1],target))

    def test_return_guard_rejects_outward_or_reexit_goals(self):
        entry=pose.reference()[2].copy();entry[10]=3301
        outward=entry.copy();outward[10]=3302
        with self.assertRaises(RuntimeError):pose.validate_goal_return(outward,entry,entry)
        too_fast=entry.copy();too_fast[10]=3298
        with self.assertRaisesRegex(RuntimeError,'2ticks'):pose.validate_goal_return(too_fast,entry,entry)
        inside=entry.copy();inside[10]=3284
        reexit=inside.copy();reexit[10]=3285
        with self.assertRaisesRegex(RuntimeError,'return toward'):pose.validate_goal_return(reexit,inside,entry)

    def test_invalid_encoder_values_still_stop(self):
        for value in (-1,4096,65535):
            start=pose.reference()[1].copy();start[10]=value
            with self.assertRaisesRegex(ValueError,'single-turn'):
                pose.path(start,pose.reference()[2])

    def test_real_write_wrapper_enforces_return_before_transmitting(self):
        calls=[]
        class Writer:
            def __init__(self,*args):self.ids=[]
            def addParam(self,motor,data):self.ids.append(motor);return True
            def txPacket(self):calls.append(self.ids.copy());return 0
            def clearParam(self):pass
        bus=object.__new__(pose.PoseBus)
        bus.sdk=type('SDK',(),{'GroupSyncWrite':Writer,'COMM_SUCCESS':0})()
        bus.port=bus.packet=object();bus.report={}
        entry=pose.reference()[2].copy();entry[10]=3301
        bus.entry_anchor=entry.copy();bus.last_goal_command=entry.copy()
        inward=entry.copy();inward[10]=3299
        bus.write(116,4,inward)
        self.assertEqual(calls,[list(range(1,17))])
        with self.assertRaisesRegex(RuntimeError,'return toward'):
            bus.write(116,4,entry)
        self.assertEqual(len(calls),1)

    def test_windows_clock_epoch_difference_uses_qpc_shared_deadline(self):
        from horizontal_quarter_health_20261027 import sample as old_sample
        from calibrated_two_full_health_20261028 import sample as fixed_sample
        bus=object.__new__(pose.PoseBus)
        bus.report={'enabled_at':time.monotonic()}
        bus.plan_module=type('Plan',(),{'NEW_PAIR_GRASP_MODE':True})()
        bus.cancel_callback=lambda:None
        clean=state(1);clean['watchdog'][:]=25
        bus.checked_control_profile={'caps':clean['current_cap'],
            'gains':{k:clean[k] for k in ('p_gain','i_gain','d_gain')}}
        legacy_epoch=time.perf_counter()-42.
        with patch('time.monotonic',return_value=legacy_epoch):
            with patch('grip_health_read_20261118.block_once',return_value=(clean,0,0,{})) as read:
                with self.assertRaisesRegex(RuntimeError,'200ms'):
                    old_sample(bus,clean['goal'],bus.report,'legacy_clock_epoch')
                self.assertEqual(read.call_count,0)
                actual,_=fixed_sample(bus,clean['goal'],bus.report,'qpc_clock_epoch')
                self.assertEqual(read.call_count,1)
                self.assertTrue(np.array_equal(actual['position'],clean['position']))

    def test_real_over_200ms_read_is_still_rejected(self):
        bus=object.__new__(pose.PoseBus)
        bus.report={}
        clean=state(1)
        def delayed(*args):
            time.sleep(.21)
            return clean,0,0,{}
        with patch('grip_health_read_20261118.block_once',side_effect=delayed):
            with self.assertRaisesRegex(RuntimeError,'200ms'):
                bus.snapshot()

    def test_live_state_uses_existing_sample_and_marks_closed(self):
        import tempfile,json
        from pathlib import Path
        from measured_grip_live_state_20261117 import LiveState
        clean=state(1)
        row=dict(phase='measured_grip_hold',t=1.,goal=clean['goal'].tolist(),actual=clean['position'].tolist(),
            torque=clean['torque'].tolist(),error=clean['error'].tolist(),current_ma=[0]*16,read_ms=28.)
        report=dict(enabled_at=time.monotonic()-1,samples=[row])
        with tempfile.TemporaryDirectory() as temp:
            writer=LiveState(temp,report)
            writer._write(writer.snapshot())
            live=json.loads((Path(temp)/'live_state.json').read_bytes())
            self.assertTrue(live['active'])
            self.assertEqual(live['actual_ticks'],row['actual'])
            closed=writer.close()
            self.assertFalse(json.loads((Path(temp)/'live_state.json').read_bytes())['active'])
            self.assertEqual(closed['motor_reads_added'],0)
            self.assertEqual(closed['motor_writes_added'],0)


if __name__=='__main__':unittest.main(verbosity=2)
