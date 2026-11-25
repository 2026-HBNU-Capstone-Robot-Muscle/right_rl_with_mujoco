"""Check exact observed path join, no intermediate OFF, gates and deadlines."""
import contextlib,io,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import leap_measured_grip_hold_20261117 as pose
from retaught_transfer_scope_20261118 import limits
import grip_rotation_return_plan_20261118 as plan
from test_measured_grip_hold_20261117 import FakeBus,Clock,state

class TraceBus(FakeBus):
    def __init__(self,s):super().__init__(s);self.goals=[];self.snapshots_at_join=0
    def snapshot(self):
        if self.report.get('phase')=='cycle_rotation_check':self.snapshots_at_join+=1
        return super().snapshot()
    def write(self,address,size,values):
        if address==116:self.goals.append((self.report.get('phase'),np.asarray(values).copy()))
        return super().write(address,size,values)

class Tests(unittest.TestCase):
    def run_sequence(self,adopt=False):
        seq=plan.GripRotationReturn();initial=state(1)
        if adopt:initial['goal']=seq.forward.taught.copy();initial['position']=seq.forward.taught.copy()
        bus=TraceBus(initial);bus.report['status']='started';clock=Clock();terminal=plan.Terminal(seq,bus.report);waits={}
        def cancel():
            phase=bus.report.get('phase')
            if phase in plan.GATES and phase not in seq.accepted:
                waits[phase]=waits.get(phase,0)+1
                if waits[phase]>=5:terminal.handle(' ')
            if bus.report.get('holding'):raise pose.OperatorStop()
        with limits(),contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(pose.OperatorStop):
                pose.run_pose(bus,initial,initial['goal'],seq.rows(initial['goal']),bus.report,cancel,
                    sequence=seq,now=clock.now,sleep=clock.sleep)
        return seq,bus,waits,clock

    def test_exact_forward_and_return_rows_are_emitted_to_same_G(self):
        seq,bus,waits,clock=self.run_sequence()
        forward=np.array([q for p,q in bus.goals if p and (p.startswith('retaught_') and p not in ('retaught_prepare','retaught_settle'))])
        back=np.array([q for p,q in bus.goals if p and p.startswith('cycle_return_')])
        self.assertTrue(np.array_equal(forward,seq.goals[:seq.join]))
        self.assertTrue(np.array_equal(back,seq.goals[seq.join:]))
        self.assertTrue(np.array_equal(bus.goals[-1][1],seq.start))
        self.assertEqual(len(back),seq.manifest['return_goals']);self.assertLess(clock.now(),500)
        self.assertEqual(set(waits),{'cycle_start_grip_check','cycle_rotation_check'})

    def test_no_OFF_until_final_fixed_grip_and_operator_exit(self):
        seq,bus,waits,_=self.run_sequence()
        last=max(i for i,(address,q) in enumerate(bus.writes) if address==116)
        off=[i for i,(address,q) in enumerate(bus.writes) if address==64 and not np.any(q)]
        self.assertTrue(off);self.assertTrue(all(i>last for i in off))
        self.assertEqual(bus.report['phase'],'cycle_fixed_grip_hold')
        self.assertTrue(bus.report['final_fixed_grip_return_included'])
        self.assertTrue(bus.report['settings_restored']);self.assertTrue(bus.report['all_torque_off_verified'])

    def test_join_wait_keeps_health_reads_and_only_SPACE_releases_return(self):
        _,bus,waits,_=self.run_sequence()
        self.assertGreaterEqual(waits['cycle_rotation_check'],5)
        self.assertGreaterEqual(bus.snapshots_at_join,4)
        self.assertEqual(len([q for p,q in bus.goals if p=='cycle_rotation_check']),1)

    def test_current_taught_entry_skips_G_return_before_the_turn(self):
        seq,bus,waits,_=self.run_sequence(adopt=True)
        self.assertTrue(seq.forward.adopted)
        self.assertEqual(set(waits),{'cycle_start_taught_check','cycle_rotation_check'})
        self.assertFalse(any(p in ('retaught_prepare','retaught_taught_clear_transfer') for p,q in bus.goals))
        self.assertTrue(np.array_equal(bus.goals[-1][1],seq.start))

    def test_fault_at_join_prevents_any_return_motion_and_cleans_up(self):
        class Fault(TraceBus):
            def snapshot(self):
                s=super().snapshot()
                if self.report.get('phase')=='cycle_rotation_check' and np.any(s['torque']):s['current'][0]=600
                return s
        seq=plan.GripRotationReturn();initial=state(1);bus=Fault(initial);bus.report['status']='started';clock=Clock();terminal=plan.Terminal(seq,bus.report)
        def cancel():
            if bus.report.get('phase') in plan.GATES:terminal.handle(' ')
        with limits(),contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError,'measured current'):
                pose.run_pose(bus,initial,initial['goal'],seq.rows(initial['goal']),bus.report,cancel,
                    sequence=seq,now=clock.now,sleep=clock.sleep)
        self.assertFalse(any(p and p.startswith('cycle_return_') for p,q in bus.goals))
        self.assertTrue(bus.report['all_torque_off_verified']);self.assertTrue(bus.report['settings_restored'])

    def test_second_confirmation_does_not_reset_global_ON_deadline(self):
        seq=plan.GripRotationReturn();initial=state(1);bus=TraceBus(initial);bus.report['status']='started';clock=Clock();terminal=plan.Terminal(seq,bus.report)
        def cancel():
            if bus.report.get('phase') in ('cycle_start_grip_check','cycle_start_taught_check'):terminal.handle(' ')
            if bus.report.get('phase')=='cycle_rotation_check':clock.sleep(501)
        with limits(),contextlib.redirect_stdout(io.StringIO()):
            pose.run_pose(bus,initial,initial['goal'],seq.rows(initial['goal']),bus.report,cancel,
                sequence=seq,now=clock.now,sleep=clock.sleep)
        self.assertEqual(bus.report['status'],'existing500second_limit_reached')
        self.assertFalse(any(p and p.startswith('cycle_return_') for p,q in bus.goals))
        self.assertTrue(bus.report['all_torque_off_verified'])

    def test_join_bounds_and500second_preparation_budget(self):
        seq=plan.GripRotationReturn()
        with limits():
            for start in (seq.start,seq.forward.taught,seq.forward.goals[-1]):
                rows=seq.rows(start);whole=np.vstack([start,rows]);_,high,low=pose.bounds()
                self.assertLessEqual(np.max(abs(np.diff(whole,axis=0))),2)
                self.assertTrue(np.all(whole>=low));self.assertTrue(np.all(whole<=high))
                self.assertLess(len(rows)*.075+30,500)
        self.assertEqual(seq.manifest['join_goal_step_ticks'],2)

    def test_tampered_join_candidate_rejected_before_device_open(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder);(p/'candidate.npz').write_bytes((plan.BASE/'candidate.npz').read_bytes()+b'changed')
            with patch.object(plan,'BASE',p):
                with self.assertRaisesRegex(RuntimeError,'candidate changed'):plan.GripRotationReturn()

    def test_only_ID12_leads_before_seating_and_original_path_catches_up(self):
        seq=plan.GripRotationReturn()
        old=np.load(plan.ROOT/'right_hand_rotation_20261118/quarter_turn_fixed_grip/tip_first_lower_seat/candidate.npz',allow_pickle=False)
        original=old['goals'][1417:];returned=seq.goals[seq.join:];labels=seq.phases[seq.join:]
        lead=labels=='return_middle_root_lead';others=[i for i in range(16) if i!=11]
        self.assertTrue(np.array_equal(returned[~lead][:,others],original[:,others]))
        self.assertEqual(returned[lead][0,11],1850);self.assertEqual(returned[lead][-1,11],1880)
        seat=labels=='return_grip_index_middle_seat'
        self.assertTrue(np.all(returned[seat,11]==1880))
        original_catch=np.flatnonzero(original[:,11]>=1880)
        catch=int(original_catch[original_catch>1517][0])
        self.assertTrue(np.array_equal(returned[~lead][catch:],original[catch:]))
        self.assertTrue(np.array_equal(returned[-1],seq.start))

    def test_early_lift_precedes_failed_ID11_goal_and_holds_other_joints(self):
        seq=plan.GripRotationReturn();returned=seq.goals[seq.join:];phases=seq.phases[seq.join:]
        lead=np.flatnonzero(phases=='return_middle_root_lead')
        seat=np.flatnonzero(phases=='return_grip_index_middle_seat')
        self.assertLess(lead[-1],seat[0])
        self.assertTrue(np.all(returned[lead,10]==2024))
        near_stop=[i for i in seat if 2559<=returned[i,10]<=2563]
        self.assertTrue(near_stop);self.assertTrue(all(returned[i,11]==1880 for i in near_stop))

if __name__=='__main__':unittest.main()
