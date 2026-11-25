"""Continuous direct return, taught endpoints, interruption and faults."""
import contextlib,io,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import leap_measured_grip_hold_20261117 as pose
import vertical_direct_grip_plan_20261118 as plan
from test_measured_grip_hold_20261117 import FakeBus,Clock,state

class Bus(FakeBus):
    def __init__(self,initial):super().__init__(initial);self.goals=[]
    def write(self,address,size,values):
        if address==116:self.goals.append((self.report.get('phase'),np.asarray(values).copy()))
        super().write(address,size,values)

class Tests(unittest.TestCase):
    def execute(self,initial=None,fault=None,hold_seconds=0):
        seq=plan.VerticalDirectGrip();initial=state(1) if initial is None else initial
        bus=Bus(initial);bus.report['status']='started';clock=Clock();terminal=plan.Terminal(seq,bus.report);seen=set()
        holding_since=None
        def cancel():
            nonlocal holding_since
            phase=bus.report.get('phase')
            if phase in plan.CHECKPOINTS and phase not in seen:
                seen.add(phase)
                if phase==fault:bus.state['current'][0]=600
            if bus.report.get('holding'):
                if holding_since is None:holding_since=clock.now()
                if clock.now()-holding_since>=hold_seconds:terminal.handle(' ')
        with contextlib.redirect_stdout(io.StringIO()):
            try:
                start=initial['goal'] if np.any(initial['torque']) else initial['position']
                pose.run_pose(bus,initial,start,seq.rows(start),bus.report,cancel,sequence=seq,
                    limit=plan.RUN_LIMIT_SECONDS,now=clock.now,sleep=clock.sleep)
            except pose.OperatorStop:pass
        return seq,bus,clock.t
    def test_source_forward_and_final_G_are_preserved(self):
        seq,bus,_=self.execute()
        forward=np.array([q for p,q in bus.goals if p and p.startswith('direct_forward_')])
        self.assertTrue(np.array_equal(forward,seq.forward.goals))
        old=np.load(plan.ROOT/seq.manifest['preserved_motion_source'],allow_pickle=False)
        kept=seq.manifest['preserved_prefix_goals']
        self.assertTrue(np.array_equal(seq.goals[:kept],old['goals'][:kept]))
        self.assertTrue(np.array_equal(seq.phases[:kept],old['phases'][:kept]))
        suffix=seq.manifest['preserved_suffix_goals'];offset=seq.manifest['suffix_source_offset']
        self.assertTrue(np.array_equal(seq.goals[-suffix:],old['goals'][offset:]))
        self.assertTrue(np.array_equal(seq.phases[-suffix:],old['phases'][offset:]))
        self.assertTrue(np.array_equal(bus.goals[-1][1],pose.reference()[2]))
        self.assertTrue(bus.report['holding']);self.assertEqual(bus.report['operator_confirmed_stages'],[])
        self.assertTrue(bus.report['automatic_stage_transitions_enabled'])
        self.assertEqual(bus.report['completed_goals'],bus.report['planned_goals'])
    def test_thumb_push_and_withdrawal_are_absent_and_lowering_is_direct(self):
        seq=plan.VerticalDirectGrip()
        lower=seq.goals[seq.phases=='direct_return_lower_index_middle_receiving'][-1]
        rows=seq.goals[seq.phases=='direct_return_thumb_lower_before_support']
        self.assertTrue(np.all(rows[:,4:12]==lower[4:12]))
        self.assertTrue(np.array_equal(rows[-1,:4],seq.start[:4]))
        self.assertTrue(np.all(rows[:,:4]>=np.minimum(lower[:4],seq.start[:4])))
        self.assertTrue(np.all(rows[:,:4]<=np.maximum(lower[:4],seq.start[:4])))
        self.assertFalse(np.any(seq.phases=='direct_return_thumb_push_toward_ring'))
        self.assertFalse(np.any(seq.phases=='direct_return_thumb_clear_cube'))
        seq.rows(seq.start)
        self.assertNotIn('direct_cube_position_check',seq.labels)
        self.assertNotIn('direct_clearance_check',seq.labels)
        self.assertFalse(any('pitch_palm_transfer' in str(p) for p in seq.phases))
    def test_thumb_lower_precedes_finger_push_on_return(self):
        seq=plan.VerticalDirectGrip()
        lower=seq.goals[seq.phases=='direct_return_thumb_lower_before_support']
        receiving=seq.goals[seq.phases=='direct_return_lower_index_middle_receiving'][-1]
        self.assertTrue(np.all(lower[:,4:12]==receiving[4:12]))
        self.assertTrue(np.array_equal(lower[-1,:4],seq.start[:4]))
        self.assertTrue(np.array_equal(lower[-1,12:],seq.start[12:]))
        phases=['direct_return_thumb_lower_before_support','direct_return_final_taught_1',
            'direct_return_final_taught_2','direct_return_final_taught_to_G']
        indices=[int(np.flatnonzero(seq.phases==p)[0]) for p in phases]
        self.assertEqual(indices,sorted(indices))
        fixed=[0,1,2,3,12,13,14,15]
        for phase in phases[1:]:
            self.assertTrue(np.all(seq.goals[seq.phases==phase][:,fixed]==seq.start[fixed]))
        for i,name in enumerate(plan.FINAL_REFS,1):
            ref=json.loads((plan.FINAL_BASE/name).read_bytes())
            target=seq.goals[seq.phases==f'direct_return_final_taught_{i}'][-1]
            self.assertTrue(np.array_equal(target[4:12],np.array(ref['holding_goal_ticks'])[4:12]))
    def test_no_OFF_until_end_and_long_final_hold_does_not_time_out(self):
        seq,bus,elapsed=self.execute(hold_seconds=600)
        last=max(i for i,(a,v) in enumerate(bus.writes) if a==116)
        self.assertTrue(all(i>last for i,(a,v) in enumerate(bus.writes) if a==64 and not np.any(v)))
        self.assertGreater(elapsed,1000);self.assertTrue(bus.report['all_torque_off_verified']);self.assertTrue(bus.report['settings_restored'])

    def test_all_stages_advance_without_operator_confirmation(self):
        seq=plan.VerticalDirectGrip();seq.rows(seq.start)
        self.assertEqual(plan.GATES,())
        self.assertFalse(seq.manifest['operator_gated_observation_enabled'])
        for i,phase in enumerate(seq.labels):
            self.assertTrue(seq.ready(i))
            self.assertNotIn('SPACE로',seq.message(phase))
        self.assertEqual(set(seq.metadata()['phase_checkpoint_indices']),set(plan.CHECKPOINTS))

    def test_space_and_cancel_stop_at_checkpoints_motion_and_hold(self):
        seq=plan.VerticalDirectGrip();seq.rows(seq.start);report={};terminal=plan.Terminal(seq,report)
        for phase in [*plan.CHECKPOINTS,'direct_return_final_taught_1','direct_fixed_grip_hold']:
            report['phase']=phase
            for key in (' ','q','Q','\x1b','\x03'):
                with self.assertRaises(pose.OperatorStop):terminal.handle(key)
        terminal.handle('x');self.assertNotIn('operator_confirmed_stages',report)
    def test_fault_after_receiving_blocks_thumb_lower_and_restores(self):
        class Capture(Bus):
            last=None
            def __init__(self,s):super().__init__(s);Capture.last=self
        with patch(__name__+'.Bus',Capture):
            with self.assertRaisesRegex(RuntimeError,'measured current'):self.execute(fault='direct_receiving_check')
        self.assertFalse(any(p=='direct_return_thumb_lower_before_support' for p,q in Capture.last.goals))
        self.assertTrue(Capture.last.report['all_torque_off_verified']);self.assertTrue(Capture.last.report['settings_restored'])
    def test_latest_recorded_clear_pose_can_enter_G_with_inward_recovery(self):
        ref=json.loads((plan.BASE/'thumb_clear_reference.json').read_bytes());raw=json.loads((plan.ROOT/ref['source']).read_bytes())
        s=state(1);snap=raw['snapshots'][-1]
        s['position']=np.array(snap['position']);s['goal']=np.array(snap['goal'])
        seq,bus,_=self.execute(initial=s)
        self.assertTrue(bus.report['holding'])
        phases=[p for p,q in bus.goals]
        self.assertLess(phases.index('direct_entry_thumb_lower'),phases.index('direct_entry_tips'))
        whole=np.vstack([s['goal'],[q for p,q in bus.goals]])
        self.assertLessEqual(np.max(abs(np.diff(whole,axis=0))),2)
    def test_recorded_entry_OFF_and_far_start_keep_guards(self):
        seq=plan.VerticalDirectGrip();low,high,native=pose.bounds()
        for current in [seq.start,np.clip(np.full(16,2048),low,high)]:
            rows=seq.rows(current);whole=np.vstack([current,rows]);self.assertLessEqual(np.max(abs(np.diff(whole,axis=0))),2)
            self.assertTrue(np.all(whole>=np.ceil(native)));self.assertTrue(np.all(whole<=high))
        seq,bus,_=self.execute(initial=state(0));self.assertTrue(bus.report['holding'])
    def test_modified_candidate_is_rejected_before_device_access(self):
        with tempfile.TemporaryDirectory() as folder:
            q=Path(folder);(q/'operator_trial_manifest.json').write_bytes((plan.BASE/'operator_trial_manifest.json').read_bytes())
            (q/'candidate.npz').write_bytes((plan.BASE/'candidate.npz').read_bytes()+b'changed')
            with patch.object(plan,'BASE',q):
                with self.assertRaisesRegex(RuntimeError,'candidate changed'):plan.VerticalDirectGrip()

    def test_actual_stopped_entry_lowers_thumb_before_other12_move(self):
        raw=json.loads((plan.ROOT/'vertical_direct_grip_trial_20261118_204744_702191/result.json').read_bytes())
        initial={k:np.asarray(v,dtype=int) for k,v in raw['initial'].items()}
        seq,bus,_=self.execute(initial=initial)
        q=np.array([q for p,q in bus.goals if p=='direct_entry_range_return'])[-1]
        thumb=np.array([q for p,q in bus.goals if p=='direct_entry_thumb_lower'])
        self.assertTrue(np.all(thumb[:,4:]==q[4:]))
        self.assertTrue(np.array_equal(thumb[-1,:4],seq.start[:4]))
        phases=[p for p,q in bus.goals]
        self.assertLess(phases.index('direct_entry_thumb_lower'),phases.index('direct_entry_tips'))
        self.assertLess(phases.index('direct_entry_tips'),phases.index('direct_entry_support'))
        self.assertTrue(bus.report['holding']);self.assertTrue(bus.report['settings_restored'])

    def test_ID2_no_progress_still_blocks_finger_push_and_restores(self):
        raw=json.loads((plan.ROOT/'vertical_direct_grip_trial_20261118_204744_702191/result.json').read_bytes())
        initial={k:np.asarray(v,dtype=int) for k,v in raw['initial'].items()}
        class Blocked(Bus):
            last=None
            def __init__(self,s):super().__init__(s);Blocked.last=self
            def write(self,address,size,values):
                previous=self.state['position'][1]
                super().write(address,size,values)
                if address==116 and self.report.get('phase')=='direct_entry_thumb_lower' and np.any(self.state['torque']):
                    self.state['position'][1]=previous
        with patch(__name__+'.Bus',Blocked):
            with self.assertRaisesRegex(RuntimeError,'ID2.*progress'):
                self.execute(initial=initial)
        self.assertFalse(any(p in ['direct_entry_tips','direct_entry_support'] for p,q in Blocked.last.goals))
        self.assertTrue(Blocked.last.report['all_torque_off_verified']);self.assertTrue(Blocked.last.report['settings_restored'])

    def test_fault_at_first_taught_waypoint_blocks_second_and_restores(self):
        class Capture(Bus):
            last=None
            def __init__(self,s):super().__init__(s);Capture.last=self
        with patch(__name__+'.Bus',Capture):
            with self.assertRaisesRegex(RuntimeError,'measured current'):
                self.execute(fault='direct_final_waypoint_1_check')
        self.assertFalse(any(p=='direct_return_final_taught_2' for p,q in Capture.last.goals))
        self.assertTrue(Capture.last.report['all_torque_off_verified']);self.assertTrue(Capture.last.report['settings_restored'])

    def test_middle_no_progress_on_final_G_still_stops_and_restores(self):
        class Blocked(Bus):
            last=None
            def __init__(self,s):super().__init__(s);Blocked.last=self
            def write(self,address,size,values):
                previous=self.state['position'][10]
                super().write(address,size,values)
                if address==116 and self.report.get('phase')=='direct_return_final_taught_to_G' and np.any(self.state['torque']):
                    self.state['position'][10]=previous
        with patch(__name__+'.Bus',Blocked):
            with self.assertRaisesRegex(RuntimeError,'ID11.*progress'):self.execute()
        self.assertFalse(Blocked.last.report.get('holding',False))
        self.assertTrue(Blocked.last.report['all_torque_off_verified']);self.assertTrue(Blocked.last.report['settings_restored'])

if __name__=='__main__':unittest.main()
