"""Check staged pitch-end return, historical sources, torque and fault gates."""
import contextlib,io,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import leap_measured_grip_hold_20261117 as pose
import vertical_return_plan_20261118 as plan
from test_measured_grip_hold_20261117 import FakeBus,Clock,state

class TraceBus(FakeBus):
    def __init__(self,s):super().__init__(s);self.goals=[]
    def write(self,address,size,values):
        if address==116:self.goals.append((self.report.get('phase'),np.asarray(values).copy()))
        return super().write(address,size,values)

class Tests(unittest.TestCase):
    def run_sequence(self):
        seq=plan.VerticalReturn();initial=state(1);bus=TraceBus(initial);bus.report['status']='started';clock=Clock();terminal=plan.Terminal(seq,bus.report);waits={}
        def cancel():
            phase=bus.report.get('phase')
            if phase in plan.GATES and phase not in seq.accepted:
                waits[phase]=waits.get(phase,0)+1
                if waits[phase]>=4:terminal.handle(' ')
            if bus.report.get('holding'):raise pose.OperatorStop()
        with contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(pose.OperatorStop):
                pose.run_pose(bus,initial,initial['goal'],seq.rows(initial['goal']),bus.report,cancel,
                    sequence=seq,now=clock.now,sleep=clock.sleep)
        return seq,bus,waits
    def test_empty_setup_precedes_placement_and_return(self):
        seq,bus,waits=self.run_sequence()
        place=seq.labels.index('vertical_return_place');first=seq.labels.index('vreturn_pitch_side_clearance')
        self.assertLess(place,first);self.assertGreaterEqual(waits['vertical_return_place'],4)
        self.assertEqual(set(waits),set(plan.GATES));self.assertTrue(all(n>=4 for n in waits.values()))
    def test_executed600_transfer_targets_and_shared_G_tail_are_exact(self):
        seq,bus,_=self.run_sequence();old=np.load(plan.TRANSFER,allow_pickle=False);h=np.load(plan.GRIP_RETURN,allow_pickle=False)
        moved=np.array([q for p,q in bus.goals if p=='vreturn_pitch_palm_transfer'])
        self.assertTrue(np.array_equal(moved,old['goals']))
        tail_phases={'vreturn_'+str(p) for p in h['phases'][seq.manifest['shared_tail_source_offset']:]}
        tail=np.array([q for p,q in bus.goals if p in tail_phases])
        self.assertTrue(np.array_equal(tail,h['goals'][seq.manifest['shared_tail_source_offset']:]))
        self.assertTrue(np.array_equal(bus.goals[-1][1],pose.reference()[2]))
    def test_pitch_is_not_replayed_and_start_is_observed_pitch_end(self):
        seq=plan.VerticalReturn();v=np.load(plan.VERTICAL,allow_pickle=False)
        self.assertTrue(np.array_equal(seq.start,v['goals'][-1]));seq.rows(seq.start)
        self.assertFalse(seq.metadata()['pitch_rotation_replayed'])
        self.assertTrue(seq.metadata()['final_fixed_grip_return_included'])
    def test_no_OFF_between_placement_and_final_fixed_grip(self):
        _,bus,_=self.run_sequence();last=max(i for i,(a,q) in enumerate(bus.writes) if a==116)
        self.assertTrue(all(i>last for i,(a,q) in enumerate(bus.writes) if a==64 and not np.any(q)))
        self.assertEqual(bus.report['phase'],'vertical_return_fixed_grip_hold')
        self.assertTrue(bus.report['settings_restored']);self.assertTrue(bus.report['all_torque_off_verified'])
    def test_middle_support_stays_taught_through_thumb_push_and_withdrawal(self):
        seq=plan.VerticalReturn();h=np.load(plan.GRIP_RETURN,allow_pickle=False)
        import json
        target=np.array(json.loads(plan.TAUGHT_PALM.read_bytes())['goal_ticks'])
        self.assertFalse(np.any(np.isin(seq.phases,['middle_clear_for_push','return_middle_root_lead'])))
        work=seq.goals[np.isin(seq.phases,['thumb_prepare_for_center','return_transfer_thumb_push','return_transfer_thumb_wrist_and_tips'])]
        self.assertTrue(np.all(work[:,8:12]==target[8:12]))
        fixed=[0,1,2,3,4,5,6,7,12,13,14,15]
        for phase in ('return_transfer_thumb_push','return_transfer_thumb_wrist_and_tips'):
            current=seq.goals[seq.phases==phase];old=h['goals'][h['phases']==phase]
            self.assertTrue(np.array_equal(current[:,fixed],old[:,fixed]))
        seat=seq.goals[seq.phases=='return_grip_index_middle_seat']
        self.assertTrue(np.array_equal(seat[-1],h['goals'][h['phases']=='return_grip_index_middle_seat'][-1]))
    def test_fault_at_palm_check_blocks_new_bridge_and_restores_OFF(self):
        class Fault(TraceBus):
            def snapshot(self):
                s=super().snapshot()
                if self.report.get('phase')=='vertical_return_palm_check' and np.any(s['torque']):s['current'][0]=600
                return s
        seq=plan.VerticalReturn();initial=state(1);bus=Fault(initial);bus.report['status']='started';clock=Clock();terminal=plan.Terminal(seq,bus.report)
        def cancel():
            if bus.report.get('phase') in plan.GATES:terminal.handle(' ')
        with contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError,'measured current'):
                pose.run_pose(bus,initial,initial['goal'],seq.rows(initial['goal']),bus.report,cancel,
                    sequence=seq,now=clock.now,sleep=clock.sleep)
        self.assertFalse(any(p in ('vreturn_thumb_prepare_for_center','vreturn_return_transfer_thumb_push') for p,q in bus.goals))
        self.assertTrue(bus.report['all_torque_off_verified']);self.assertTrue(bus.report['settings_restored'])
    def test_original_bounds_two_tick_step_and500second_budget(self):
        seq=plan.VerticalReturn();_,high,low=pose.bounds()
        for start in (seq.start,seq.grip):
            rows=seq.rows(start);whole=np.vstack([start,rows])
            self.assertLessEqual(np.max(abs(np.diff(whole,axis=0))),2)
            self.assertTrue(np.all(whole>=low));self.assertTrue(np.all(whole<=high))
            self.assertLess(len(rows)*.075+30,500)
    def test_tampered_candidate_rejected_before_device_access(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder);(p/'candidate.npz').write_bytes((plan.BASE/'candidate.npz').read_bytes()+b'changed')
            with patch.object(plan,'BASE',p):
                with self.assertRaisesRegex(RuntimeError,'candidate changed'):plan.VerticalReturn()

    def test_center_confirmation_holds_push_goal_before_receiving_and_closing(self):
        seq=plan.VerticalReturn();rows=seq.rows(seq.start)
        center=seq.labels.index('vertical_return_center_check')
        after=seq.labels.index('vreturn_return_transfer_thumb_wrist_and_tips')
        self.assertLess(center,after);self.assertFalse(seq.ready(center))
        self.assertTrue(np.array_equal(rows[center],rows[center-1]))
        self.assertEqual(seq.metadata()['required_cube_center'],'between_index_and_middle')

    def test_first_palm_check_uses_new_taught_goal_before_thumb_preparation(self):
        import json
        seq=plan.VerticalReturn();rows=seq.rows(seq.start)
        target=np.array(json.loads(plan.TAUGHT_PALM.read_bytes())['goal_ticks'])
        check=seq.labels.index('vertical_return_palm_check')
        self.assertTrue(np.array_equal(rows[check],target))
        self.assertTrue(np.array_equal(rows[check-1],target))
        self.assertLess(check,seq.labels.index('vreturn_thumb_prepare_for_center'))
        self.assertFalse(seq.ready(check))

    def test_fault_at_center_check_prevents_grip_completion_and_restores_OFF(self):
        class Fault(TraceBus):
            def snapshot(self):
                s=super().snapshot()
                if self.report.get('phase')=='vertical_return_center_check' and np.any(s['torque']):s['current'][0]=600
                return s
        seq=plan.VerticalReturn();initial=state(1);bus=Fault(initial);bus.report['status']='started';clock=Clock();terminal=plan.Terminal(seq,bus.report)
        def cancel():
            if bus.report.get('phase') in plan.GATES:terminal.handle(' ')
        with contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError,'measured current'):
                pose.run_pose(bus,initial,initial['goal'],seq.rows(initial['goal']),bus.report,cancel,
                    sequence=seq,now=clock.now,sleep=clock.sleep)
        self.assertFalse(any(p in ('vreturn_return_transfer_thumb_wrist_and_tips','vreturn_return_fixed_grip_close') for p,q in bus.goals))
        self.assertTrue(bus.report['all_torque_off_verified']);self.assertTrue(bus.report['settings_restored'])

if __name__=='__main__':unittest.main()
