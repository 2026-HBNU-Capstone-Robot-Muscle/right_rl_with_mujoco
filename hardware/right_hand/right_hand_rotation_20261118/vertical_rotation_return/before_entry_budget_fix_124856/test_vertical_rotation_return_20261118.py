"""Device-free connected execution: sources, transition, torque and deadline."""
import contextlib,io,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import leap_measured_grip_hold_20261117 as pose
import vertical_rotation_return_plan_20261118 as plan
from test_measured_grip_hold_20261117 import FakeBus,Clock,state

class TraceBus(FakeBus):
    def __init__(self,initial):super().__init__(initial);self.goals=[]
    def write(self,address,size,values):
        if address==116:self.goals.append((self.report.get('phase'),np.asarray(values).copy()))
        return super().write(address,size,values)

class Tests(unittest.TestCase):
    def execute(self,initial=None,bus_type=TraceBus,advance_gates=True):
        seq=plan.VerticalRotationReturn();initial=state(1) if initial is None else initial
        bus=bus_type(initial);bus.report['status']='started';clock=Clock();terminal=plan.Terminal(seq,bus.report);waits={}
        def cancel():
            phase=bus.report.get('phase')
            if phase in plan.GATES and phase not in seq.accepted:
                waits[phase]=waits.get(phase,0)+1
                if advance_gates and waits[phase]>=4:terminal.handle(' ')
            if bus.report.get('holding'):raise pose.OperatorStop()
        with contextlib.redirect_stdout(io.StringIO()):
            try:
                pose.run_pose(bus,initial,initial['goal'] if np.any(initial['torque']) else initial['position'],
                    seq.rows(initial['goal'] if np.any(initial['torque']) else initial['position']),
                    bus.report,cancel,sequence=seq,now=clock.now,sleep=clock.sleep)
            except pose.OperatorStop:pass
        return seq,bus,waits,clock.t
    def test_all_moving_targets_preserved_and_join_endpoints_match(self):
        seq,bus,waits,_=self.execute()
        f=np.array([q for p,q in bus.goals if p and p.startswith('cycle_forward_')])
        r=np.array([q for p,q in bus.goals if p and p.startswith('cycle_return_')])
        self.assertTrue(np.array_equal(f,seq.forward.goals))
        self.assertTrue(np.array_equal(r,seq.returned.goals[:seq.manifest['return_goals']]))
        self.assertTrue(np.array_equal(f[-1],seq.returned.start))
        self.assertEqual(seq.manifest['join_endpoint_mismatch_ticks'],0)
        self.assertEqual(seq.manifest['join_step_ticks'],int(np.max(abs(r[0]-f[-1]))))
        self.assertLessEqual(seq.manifest['join_step_ticks'],2)
        self.assertTrue(np.array_equal(r[-1],seq.start));self.assertEqual(set(waits),set(plan.GATES))
    def test_no_torque_OFF_before_final_G_and_no_midway_reset(self):
        seq,bus,_,_=self.execute();last=max(i for i,(a,v) in enumerate(bus.writes) if a==116)
        self.assertTrue(all(i>last for i,(a,v) in enumerate(bus.writes) if a==64 and not np.any(v)))
        self.assertEqual(bus.report['phase'],'vertical_cycle_fixed_grip_hold');self.assertTrue(bus.report['holding'])
        self.assertTrue(bus.report['all_torque_off_verified']);self.assertTrue(bus.report['settings_restored'])
        self.assertEqual(len([1 for a,v in bus.writes if a==102 and np.all(v==450)]),2)
    def test_join_gate_holds_pitch_end_before_return(self):
        seq=plan.VerticalRotationReturn();rows=seq.rows(seq.start);i=seq.labels.index(plan.GATES[1])
        self.assertFalse(seq.ready(i));self.assertTrue(np.array_equal(rows[i],rows[i-1]))
        self.assertTrue(np.array_equal(rows[i],seq.returned.start))
        self.assertEqual(seq.labels[i+1],'cycle_return_pitch_side_clearance')
    def test_fault_at_join_blocks_return_and_restores(self):
        class Fault(TraceBus):
            last=None
            def __init__(self,initial):super().__init__(initial);Fault.last=self
            def snapshot(self):
                s=super().snapshot()
                if self.report.get('phase')==plan.GATES[1] and np.any(s['torque']):s['current'][0]=600
                return s
        with self.assertRaisesRegex(RuntimeError,'measured current'):self.execute(bus_type=Fault)
        self.assertFalse(any(p and p.startswith('cycle_return_') for p,q in Fault.last.goals))
        self.assertTrue(Fault.last.report['all_torque_off_verified']);self.assertTrue(Fault.last.report['settings_restored'])
    def test_global500second_deadline_covers_unanswered_gate(self):
        _,bus,waits,elapsed=self.execute(advance_gates=False)
        self.assertEqual(bus.report['status'],'existing500second_limit_reached');self.assertFalse(bus.report.get('holding',False))
        self.assertFalse(any(p and p.startswith('cycle_forward_') for p,q in bus.goals))
        self.assertGreaterEqual(elapsed,500);self.assertTrue(bus.report['all_torque_off_verified'])
    def test_bounds_step_and_budget_checked_before_start(self):
        seq=plan.VerticalRotationReturn();low,high,native=pose.bounds()
        for current in (seq.start,pose.reference()[1]):
            rows=seq.rows(current);whole=np.vstack([current,rows])
            self.assertLessEqual(np.max(abs(np.diff(whole,axis=0))),2)
            self.assertTrue(np.all(whole>=np.ceil(native)));self.assertTrue(np.all(whole<=high))
            self.assertLess(len(rows)*.075+30,500)
        with self.assertRaisesRegex(RuntimeError,'500second budget'):seq.rows(np.clip(np.full(16,2048),low,high))
    def test_only_final_constant_dwell_is_replaced_by_live_hold(self):
        seq=plan.VerticalRotationReturn();n=seq.manifest['return_goals']
        self.assertEqual(seq.manifest['final_constant_hold_rows_replaced_by_runtime_hold'],134)
        self.assertTrue(np.all(seq.returned.goals[n:]==seq.start))
        self.assertTrue(np.all(seq.returned.phases[n:]=='return_fixed_grip_hold'))
    def test_middle_support_stays_taught_and_old_retreat_is_absent(self):
        seq=plan.VerticalRotationReturn();target=seq.returned.manifest['taught_palm_target_ticks']
        phases=['cycle_return_'+p for p in ['thumb_prepare_for_center','return_transfer_thumb_push','return_transfer_thumb_wrist_and_tips']]
        self.assertTrue(np.all(seq.goals[np.isin(seq.phases,phases),8:12]==np.array(target)[8:12]))
        self.assertFalse(any('middle_clear_for_push' in str(p) for p in seq.phases))
    def test_OFF_start_preserves_fresh_preload_and_all_source_goals(self):
        seq,bus,_,_=self.execute(initial=state(0))
        first=next(v for a,v in bus.writes if a==116)
        self.assertTrue(np.array_equal(first,pose.reference()[1]));self.assertTrue(bus.report['holding'])
        self.assertEqual(len([1 for a,v in bus.writes if a==64 and np.all(v==1)]),1)
    def test_candidate_tampering_rejected_before_device_access(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder);(p/'candidate.npz').write_bytes((plan.BASE/'candidate.npz').read_bytes()+b'changed')
            (p/'operator_trial_manifest.json').write_bytes((plan.BASE/'operator_trial_manifest.json').read_bytes())
            with patch.object(plan,'BASE',p):
                with self.assertRaisesRegex(RuntimeError,'candidate changed'):plan.VerticalRotationReturn()

if __name__=='__main__':unittest.main()
