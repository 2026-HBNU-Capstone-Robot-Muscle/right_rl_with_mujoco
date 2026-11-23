"""Verify vertical axis source, taught pose adoption, handover and fault gates."""
import contextlib,io,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import leap_measured_grip_hold_20261117 as pose
import vertical_transfer_plan_20261118 as plan
from test_measured_grip_hold_20261117 import FakeBus,Clock,state
from hand_motor_mapping import SIM_TO_MOTOR_IDS

class TraceBus(FakeBus):
    def __init__(self,s):super().__init__(s);self.goals=[]
    def write(self,address,size,values):
        if address==116:self.goals.append((self.report.get('phase'),np.asarray(values).copy()))
        return super().write(address,size,values)

class Tests(unittest.TestCase):
    def run_sequence(self,adopt=False):
        seq=plan.VerticalTransfer();initial=state(1)
        if adopt:initial['goal']=seq.taught.copy();initial['position']=seq.taught.copy()
        bus=TraceBus(initial);bus.report['status']='started';clock=Clock();terminal=plan.Terminal(seq,bus.report);waits={}
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
    def test_recorded_all16_goals_are_the_vertical_reference_without_clipping(self):
        seq=plan.VerticalTransfer()
        target=[2020,3173,2574,2990,1937,2066,3145,1971,1867,2055,3178,1981,2911,2227,3203,1879]
        self.assertEqual(seq.taught.tolist(),target)
        self.assertEqual(seq.goals[seq.taught_end].tolist(),target)
    def test_raw_pitch1268_preserved_and_reordered_variant_keeps_end_and_tail(self):
        seq=plan.VerticalTransfer();old=np.load(plan.PITCH,allow_pickle=False)
        self.assertTrue(np.array_equal(old['motor_ids_in_sim_order'],SIM_TO_MOTOR_IDS))
        self.assertEqual(float(old['sample_period_s']),.075)
        pitch=seq.goals[seq.preparation:]
        self.assertEqual(len(old['goals']),1268);self.assertEqual(len(pitch),1507)
        self.assertTrue(np.array_equal(pitch[-1],old['goals'][-1]))
        self.assertTrue(np.array_equal(pitch[-252:],old['goals'][1016:]))
    def test_current_vertical_pose_is_adopted_without_G_detour(self):
        seq,bus,waits=self.run_sequence(adopt=True)
        self.assertTrue(bus.report['adopted_current_taught_pose'])
        self.assertFalse(any(p in ('vertical_prepare','vertical_lift_index_middle','vertical_thumb_push_to_vertical') for p,q in bus.goals))
        self.assertNotIn('vertical_grip_check',waits)
        self.assertEqual(bus.report['phase'],'vertical_rotation_hold')
    def test_from_G_visits_new_pose_and_runs_old_pitch(self):
        seq,bus,waits=self.run_sequence()
        moved=np.array([q for p,q in bus.goals if p=='vertical_thumb_push_to_vertical'])
        self.assertTrue(np.array_equal(moved[-1],seq.taught))
        emitted=np.array([q for p,q in bus.goals if p and p.startswith('vertical_pitch_') and p not in plan.GATES])
        self.assertTrue(np.array_equal(emitted,seq.goals[seq.preparation:]))
        self.assertTrue(all(n>=4 for n in waits.values()))
    def test_support_handover_holds_the_other_fingers(self):
        seq=plan.VerticalTransfer()
        support=seq.goals[seq.phases=='thumb_ring_to_pitch_start']
        self.assertTrue(np.all(support[:,4:12]==seq.taught[4:12]))
        opened=seq.goals[seq.phases=='index_middle_to_pitch_start'];fixed=[0,1,2,3,12,13,14,15]
        self.assertTrue(np.all(opened[:,fixed]==support[-1,fixed]))
    def test_no_OFF_between_taught_pose_and_pitch_hold(self):
        _,bus,_=self.run_sequence(adopt=True)
        last=max(i for i,(a,q) in enumerate(bus.writes) if a==116)
        self.assertTrue(all(i>last for i,(a,q) in enumerate(bus.writes) if a==64 and not np.any(q)))
        self.assertTrue(bus.report['settings_restored']);self.assertTrue(bus.report['all_torque_off_verified'])
    def test_fault_at_pitch_start_blocks_the_rotation(self):
        class Fault(TraceBus):
            def snapshot(self):
                s=super().snapshot()
                if self.report.get('phase')=='vertical_pitch_start_check' and np.any(s['torque']):s['current'][0]=600
                return s
        seq=plan.VerticalTransfer();initial=state(1);bus=Fault(initial);bus.report['status']='started';clock=Clock();terminal=plan.Terminal(seq,bus.report)
        def cancel():
            if bus.report.get('phase') in plan.GATES:terminal.handle(' ')
        with contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError,'measured current'):
                pose.run_pose(bus,initial,initial['goal'],seq.rows(initial['goal']),bus.report,cancel,
                    sequence=seq,now=clock.now,sleep=clock.sleep)
        self.assertFalse(any(p and p.startswith('vertical_pitch_') and p not in plan.GATES for p,q in bus.goals))
        self.assertTrue(bus.report['all_torque_off_verified']);self.assertTrue(bus.report['settings_restored'])
    def test_two_tick_bound_and_original_joint_limits_and500second_budget(self):
        seq=plan.VerticalTransfer();_,high,low=pose.bounds()
        for current in (seq.start,seq.taught):
            rows=seq.rows(current);whole=np.vstack([current,rows])
            self.assertLessEqual(np.max(abs(np.diff(whole,axis=0))),2)
            self.assertTrue(np.all(whole>=low));self.assertTrue(np.all(whole<=high))
            self.assertLess(len(rows)*.075+30,500)
    def test_tampered_candidate_is_rejected_before_device_access(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder);(p/'candidate.npz').write_bytes((plan.BASE/'candidate.npz').read_bytes()+b'changed')
            with patch.object(plan,'BASE',p):
                with self.assertRaisesRegex(RuntimeError,'candidate changed'):plan.VerticalTransfer()
    def test_SPACE_during_movement_stops_and_return_G_is_not_claimed(self):
        seq=plan.VerticalTransfer();terminal=plan.Terminal(seq,{'phase':'vertical_thumb_ring_to_pitch_start'})
        with self.assertRaises(pose.OperatorStop):terminal.handle(' ')
        seq.rows(seq.start);self.assertFalse(seq.metadata()['final_fixed_grip_return_included'])

    def test_index_middle_clear_first_then_thumb_and_ring_move_with_clearance(self):
        seq=plan.VerticalTransfer();g=seq.start
        lifted=seq.goals[seq.phases=='lift_index_middle'];fixed=[0,1,2,3,12,13,14,15]
        self.assertTrue(np.all(lifted[:,fixed]==g[fixed]))
        self.assertTrue(np.array_equal(lifted[-1,4:12],seq.taught[4:12]))
        pushed=seq.goals[seq.phases=='thumb_push_to_vertical']
        self.assertTrue(np.all(pushed[:,4:12]==lifted[-1,4:12]))
        self.assertTrue(np.array_equal(pushed[-1],seq.taught))
        self.assertFalse(np.array_equal(pushed[-1,12:16],lifted[-1,12:16]))
        seq.rows(seq.start)
        self.assertLess(seq.labels.index('vertical_lift_check'),seq.labels.index('vertical_thumb_push_to_vertical'))
        self.assertLess(seq.labels.index('vertical_thumb_push_to_vertical'),seq.labels.index('vertical_relocate_check'))

    def test_forced_G_entry_replays_new_order_even_if_initial_goal_is_taught_pose(self):
        seq=plan.VerticalTransfer(force_grip=True);rows=seq.rows(seq.taught)
        self.assertFalse(seq.adopted);self.assertTrue(seq.metadata()['forced_grip_entry'])
        self.assertIn('vertical_prepare',seq.labels);self.assertIn('vertical_grip_check',seq.labels)
        self.assertIn('vertical_lift_index_middle',seq.labels)

    def test_fault_after_finger_clear_blocks_thumb_push_and_restores_OFF(self):
        class Fault(TraceBus):
            def snapshot(self):
                s=super().snapshot()
                if self.report.get('phase')=='vertical_lift_check' and np.any(s['torque']):s['current'][0]=600
                return s
        seq=plan.VerticalTransfer(force_grip=True);initial=state(1);bus=Fault(initial);bus.report['status']='started';clock=Clock();terminal=plan.Terminal(seq,bus.report)
        def cancel():
            if bus.report.get('phase') in plan.GATES:terminal.handle(' ')
        with contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError,'measured current'):
                pose.run_pose(bus,initial,initial['goal'],seq.rows(initial['goal']),bus.report,cancel,
                    sequence=seq,now=clock.now,sleep=clock.sleep)
        self.assertFalse(any(p=='vertical_thumb_push_to_vertical' for p,q in bus.goals))
        self.assertTrue(bus.report['settings_restored']);self.assertTrue(bus.report['all_torque_off_verified'])

    def test_thumb_finishes_lowering_before_index_middle_roots_rise(self):
        seq=plan.VerticalTransfer();old=np.load(plan.PITCH,allow_pickle=False);pitch=seq.goals[seq.preparation:]
        roots=[7,11];others=[i for i in range(16) if i not in roots]
        self.assertTrue(np.all(pitch[:952,roots]==old['start'][roots]))
        self.assertTrue(np.array_equal(pitch[:760,others],old['goals'][:760,others]))
        self.assertTrue(np.array_equal(pitch[760:952,others],old['goals'][824:1016,others]))
        self.assertEqual(pitch[951,3],2423)
        self.assertTrue(np.all(pitch[952:,:4]==old['goals'][1015,:4]))
        self.assertTrue(np.all(pitch[952:1255,others]==old['goals'][1015,others]))
        seq.rows(seq.start)
        self.assertLess(seq.labels.index('vertical_thumb_lowered_check'),seq.labels.index('vertical_pitch_index_middle_raise'))

    def test_fault_after_thumb_lowering_blocks_root_raise_and_restores_OFF(self):
        class Fault(TraceBus):
            def snapshot(self):
                s=super().snapshot()
                if self.report.get('phase')=='vertical_thumb_lowered_check' and np.any(s['torque']):s['current'][3]=600
                return s
        seq=plan.VerticalTransfer(force_grip=True);initial=state(1);bus=Fault(initial);bus.report['status']='started';clock=Clock();terminal=plan.Terminal(seq,bus.report)
        def cancel():
            if bus.report.get('phase') in plan.GATES:terminal.handle(' ')
        with contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError,'measured current'):
                pose.run_pose(bus,initial,initial['goal'],seq.rows(initial['goal']),bus.report,cancel,
                    sequence=seq,now=clock.now,sleep=clock.sleep)
        self.assertFalse(any(p=='vertical_pitch_index_middle_raise' for p,q in bus.goals))
        self.assertTrue(bus.report['settings_restored']);self.assertTrue(bus.report['all_torque_off_verified'])

if __name__=='__main__':unittest.main()
