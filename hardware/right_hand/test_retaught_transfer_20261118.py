"""Verify exact retaught goals, scoped limits, current-pose adoption and gates."""
import contextlib,io,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import leap_measured_grip_hold_20261117 as pose
import retaught_transfer_scope_20261118 as scope
import retaught_transfer_plan_20261118 as plan
from test_measured_grip_hold_20261117 import FakeBus,Clock,state

class TraceBus(FakeBus):
    def __init__(self,s):super().__init__(s);self.goals=[]
    def write(self,address,size,values):
        if address==116:self.goals.append((self.report.get('phase'),np.asarray(values).copy()))
        return super().write(address,size,values)

class Tests(unittest.TestCase):
    def run_sequence(self,adopted=False,transfer_only=False,bus_type=TraceBus):
        seq=plan.RetaughtTransfer(transfer_only);initial=state(1)
        if adopted:initial['goal']=seq.taught.copy();initial['position']=seq.taught.copy()
        bus=bus_type(initial);bus.report['status']='started';clock=Clock();terminal=plan.Terminal(seq,bus.report);waits={}
        def cancel():
            phase=bus.report.get('phase')
            if phase in plan.GATES and phase not in seq.accepted:
                waits[phase]=waits.get(phase,0)+1
                if waits[phase]>=3:terminal.handle(' ')
            if bus.report.get('holding'):raise pose.OperatorStop()
        with scope.limits(),contextlib.redirect_stdout(io.StringIO()):
            rows=seq.rows(initial['goal'])
            with self.assertRaises(pose.OperatorStop):
                pose.run_pose(bus,initial,initial['goal'],rows,bus.report,cancel,
                    sequence=seq,now=clock.now,sleep=clock.sleep,limit=490.)
        return seq,bus,waits

    def test_exact_all_ON_goals_used_without_clipping_or_old_clearance(self):
        seq=plan.RetaughtTransfer()
        expected=[2538,2814,2883,2321,3141,2011,3149,1818,3165,2209,3191,1833,2790,3009,3254,2600]
        self.assertEqual(seq.taught.tolist(),expected)
        self.assertEqual(seq.goals[seq.taught_end].tolist(),expected)
        self.assertEqual(set(seq.phases[:seq.taught_end+1]),{'taught_clear_transfer'})
        self.assertFalse(any('ring_push' in str(p) or 'thumb_clear_for_turn' in str(p) for p in seq.phases))

    def test_scope_extends_only_exact_recorded_three_joint_targets(self):
        _,target=scope.reference();before=pose.bounds();original=pose.bounds
        with scope.limits():
            inside=pose.bounds()
            changed=[i+1 for i in range(16) if inside[1][i]!=before[1][i] or inside[2][i]!=before[2][i]]
            self.assertEqual(changed,[8,12,14]);self.assertEqual(inside[1][13],3009)
            invalid=target.copy();invalid[13]+=1
            with self.assertRaises(RuntimeError):pose.validate_goal_return(invalid,target,target)
        self.assertIs(pose.bounds,original)

    def test_scope_restored_after_exception(self):
        original=pose.bounds
        with self.assertRaisesRegex(RuntimeError,'test exit'):
            with scope.limits():raise RuntimeError('test exit')
        self.assertIs(pose.bounds,original)

    def test_current_taught_pose_is_adopted_without_return_to_G_and_stays_ON(self):
        seq,bus,waits=self.run_sequence(adopted=True)
        self.assertTrue(bus.report['adopted_current_taught_pose'])
        self.assertFalse(any(p in ('retaught_prepare','retaught_taught_clear_transfer') for p,q in bus.goals))
        self.assertNotIn('retaught_grip_check',waits)
        self.assertTrue(all(n>=3 for n in waits.values()))
        last=max(i for i,(a,q) in enumerate(bus.writes) if a==116)
        self.assertTrue(all(i>last for i,(a,q) in enumerate(bus.writes) if a==64 and not np.any(q)))
        self.assertTrue(bus.report['all_torque_off_verified']);self.assertTrue(bus.report['settings_restored'])

    def test_from_G_visits_new_pose_then_preserves_exact_full_turn(self):
        seq,bus,waits=self.run_sequence()
        self.assertFalse(seq.adopted);self.assertIn('retaught_transfer_check',waits)
        end=np.array([q for p,q in bus.goals if p=='retaught_taught_clear_transfer'])[-1]
        self.assertTrue(np.array_equal(end,seq.taught))
        turn=np.array([q for p,q in bus.goals if p and p.startswith('retaught_preserved_')])
        self.assertTrue(np.array_equal(turn,seq.goals[seq.preparation:]));self.assertEqual(len(turn),772)

    def test_both_entries_can_stop_and_hold_at_new_pose_without_turn(self):
        for adopted in (False,True):
            seq,bus,_=self.run_sequence(adopted=adopted,transfer_only=True)
            self.assertTrue(np.array_equal(bus.goals[-1][1],seq.taught))
            self.assertFalse(any(p and ('preserved_' in p or p=='retaught_release_index') for p,q in bus.goals))
            self.assertTrue(bus.report['all_torque_off_verified'])

    def test_all_targets_keep_two_tick_bound_and_exact_scope(self):
        seq=plan.RetaughtTransfer()
        with scope.limits():
            for current in (seq.start,seq.taught):
                rows=seq.rows(current);whole=np.vstack([current,rows]);_,high,low=pose.bounds()
                self.assertLessEqual(np.max(abs(np.diff(whole,axis=0))),2)
                self.assertTrue(np.all(whole>=low));self.assertTrue(np.all(whole<=high))

    def test_fault_before_rotation_blocks_full_turn_and_restores_OFF(self):
        class Fault(TraceBus):
            def snapshot(self):
                s=super().snapshot()
                if self.report.get('phase')=='retaught_ready_check' and np.any(s['torque']):s['current'][13]=600
                return s
        seq=plan.RetaughtTransfer();initial=state(1);initial['goal']=seq.taught.copy();initial['position']=seq.taught.copy()
        bus=Fault(initial);bus.report['status']='started';terminal=plan.Terminal(seq,bus.report);clock=Clock()
        def cancel():
            if bus.report.get('phase') in plan.GATES:terminal.handle(' ')
        with scope.limits(),contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError,'measured current'):
                pose.run_pose(bus,initial,initial['goal'],seq.rows(initial['goal']),bus.report,cancel,
                    sequence=seq,now=clock.now,sleep=clock.sleep,limit=490.)
        self.assertFalse(any(p and 'preserved_' in p for p,q in bus.goals))
        self.assertTrue(bus.report['all_torque_off_verified']);self.assertTrue(bus.report['settings_restored'])

    def test_tampered_candidate_rejected_before_device_access(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder);(p/'candidate.npz').write_bytes((scope.BASE/'candidate.npz').read_bytes()+b'tampered')
            with patch.object(plan,'BASE',p):
                with self.assertRaisesRegex(RuntimeError,'candidate changed'):plan.RetaughtTransfer()

    def test_runtime_requires_scope_and_SPACE_during_motion_stops(self):
        seq=plan.RetaughtTransfer()
        with self.assertRaisesRegex(RuntimeError,'scope'):seq.rows(seq.start)
        terminal=plan.Terminal(seq,{'phase':'retaught_thumb_catch'})
        with self.assertRaises(pose.OperatorStop):terminal.handle(' ')

if __name__=='__main__':unittest.main()
