"""One-bus compound trajectory, same-G join, scoped limits and failures."""
import contextlib,io,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import leap_measured_grip_hold_20261117 as pose
import two_axis_cycle_plan_20261124 as plan
from test_measured_grip_hold_20261117 import FakeBus,Clock,state

class Bus(FakeBus):
    def __init__(self,initial):super().__init__(initial);self.goals=[]
    def write(self,address,size,values):
        if address==116:self.goals.append((self.report.get('phase'),np.asarray(values).copy()))
        super().write(address,size,values)

class Tests(unittest.TestCase):
    def execute(self,initial=None,fault=None,cancel_at_handoff=False):
        seq=plan.TwoAxisCycle();initial=state(1) if initial is None else initial
        bus=Bus(initial);self.last_bus=bus;bus.report.update(status='started',holding=False)
        clock=Clock();terminal=plan.Terminal();seen=set()
        def cancel():
            phase=bus.report.get('phase')
            if fault is not None and phase==fault and phase not in seen:
                bus.state['current'][0]=600;seen.add(phase)
            if cancel_at_handoff and phase=='chain_handoff_G_check':terminal.handle(' ')
            if bus.report.get('holding'):terminal.handle(' ')
        with seq.runtime_limits(),contextlib.redirect_stdout(io.StringIO()):
            try:
                start=initial['goal'] if np.any(initial['torque']) else initial['position']
                pose.run_pose(bus,initial,start,seq.rows(start),bus.report,cancel,
                    sequence=seq,limit=plan.RUN_LIMIT_SECONDS,now=clock.now,sleep=clock.sleep)
            except pose.OperatorStop:pass
        return seq,bus,clock.t

    def test_both_exact_components_run_past500s_without_OFF_or_operator_gates(self):
        seq,bus,elapsed=self.execute()
        h=np.array([q for p,q in bus.goals if p and p.startswith('chain_h_')])
        v=np.array([q for p,q in bus.goals if p and p.startswith('chain_v_')])
        self.assertTrue(np.array_equal(h,seq.horizontal.goals))
        self.assertTrue(np.array_equal(v,seq.vertical.goals))
        self.assertTrue(np.array_equal(bus.goals[-1][1],seq.start))
        self.assertGreater(elapsed,800);self.assertTrue(bus.report['holding'])
        self.assertEqual(bus.report['operator_confirmed_stages'],[])
        last_goal=max(i for i,(a,_) in enumerate(bus.writes) if a==116)
        self.assertTrue(all(i>last_goal for i,(a,v) in enumerate(bus.writes) if a==64 and not np.any(v)))
        self.assertTrue(bus.report['all_torque_off_verified']);self.assertTrue(bus.report['settings_restored'])
        self.assertIs(pose.bounds,plan.ORIGINAL_BOUNDS)

    def test_G_handoff_and_existing_ten_second_hold_are_exact(self):
        seq=plan.TwoAxisCycle();rows=seq.rows(seq.start)
        self.assertTrue(np.array_equal(rows[seq.handoff_index],seq.start))
        self.assertTrue(np.array_equal(rows[seq.h_end-1],seq.start))
        hold=seq.horizontal.goals[seq.horizontal.phases=='return_fixed_grip_hold']
        self.assertTrue(np.all(hold==seq.start));self.assertEqual(len(hold),134)
        self.assertAlmostEqual(seq.metadata()['horizontal_G_hold_seconds'],10.05)
        self.assertLessEqual(np.max(abs(rows[seq.v_begin]-rows[seq.handoff_index])),2)
        self.assertTrue(all(seq.ready(i) for i in range(len(rows))))
        for phase in set(seq.labels):self.assertNotIn('SPACE로',seq.message(phase))

    def test_horizontal_limit_exception_does_not_extend_into_vertical(self):
        seq=plan.TwoAxisCycle();seq.rows(seq.start);native=plan.ORIGINAL_BOUNDS()
        with seq.runtime_limits():
            seq.phase(seq.h_begin,True)
            self.assertTrue(all(np.array_equal(a,b) for a,b in zip(pose.bounds(),seq.h_bounds)))
            self.assertGreater(pose.bounds()[1][13],native[1][13])
            seq.phase(seq.handoff_index,True)
            self.assertTrue(all(np.array_equal(a,b) for a,b in zip(pose.bounds(),native)))
            seq.phase(seq.v_begin,True)
            self.assertTrue(all(np.array_equal(a,b) for a,b in zip(pose.bounds(),native)))
        self.assertIs(pose.bounds,plan.ORIGINAL_BOUNDS)

    def test_fault_at_G_handoff_blocks_vertical_and_restores(self):
        with self.assertRaisesRegex(RuntimeError,'measured current'):
            self.execute(fault='chain_handoff_G_check')
        bus=self.last_bus
        self.assertFalse(any(p and p.startswith('chain_v_') for p,q in bus.goals))
        self.assertTrue(bus.report['all_torque_off_verified']);self.assertTrue(bus.report['settings_restored'])
        self.assertIs(pose.bounds,plan.ORIGINAL_BOUNDS)

    def test_fault_during_vertical_does_not_claim_final_hold(self):
        with self.assertRaisesRegex(RuntimeError,'measured current'):
            self.execute(fault='chain_v_direct_return_final_taught_1')
        bus=self.last_bus
        self.assertFalse(bus.report['holding'])
        self.assertFalse(any(p=='chain_final_G_check' for p,q in bus.goals))
        self.assertTrue(bus.report['all_torque_off_verified']);self.assertTrue(bus.report['settings_restored'])
        self.assertIs(pose.bounds,plan.ORIGINAL_BOUNDS)

    def test_OFF_start_enables_once_and_has_no_midcycle_OFF(self):
        seq,bus,_=self.execute(initial=state(0))
        enables=[v for a,v in bus.writes if a==64 and np.all(v)]
        self.assertEqual(len(enables),1);self.assertTrue(bus.report['holding'])
        self.assertEqual(sum(a==64 and not np.any(v) for a,v in bus.writes),1)
        self.assertEqual(sum(a==102 for a,v in bus.writes),2)

    def test_stop_key_at_handoff_does_not_start_vertical(self):
        seq,bus,_=self.execute(cancel_at_handoff=True)
        self.assertFalse(any(p and p.startswith('chain_v_') for p,q in bus.goals))
        self.assertTrue(bus.report['all_torque_off_verified']);self.assertTrue(bus.report['settings_restored'])

    def test_tampered_candidate_is_rejected_before_devices(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)
            (p/'operator_trial_manifest.json').write_bytes((plan.BASE/'operator_trial_manifest.json').read_bytes())
            (p/'candidate.npz').write_bytes((plan.BASE/'candidate.npz').read_bytes()+b'changed')
            with patch.object(plan,'BASE',p):
                with self.assertRaisesRegex(RuntimeError,'candidate changed'):plan.TwoAxisCycle()

    def test_current_entry_is_bounded_and_final_G_is_same(self):
        seq=plan.TwoAxisCycle();low,high,_=pose.bounds()
        for current in (seq.start,np.clip(np.full(16,2048),low,high),seq.horizontal.forward.taught):
            rows=seq.rows(current)
            self.assertLessEqual(np.max(abs(np.diff(np.vstack([current,rows]),axis=0))),2)
            self.assertTrue(np.array_equal(rows[-1],seq.start))

    def test_all_runtime_cancel_keys_stop(self):
        terminal=plan.Terminal()
        for key in (' ','q','Q','\x1b','\x03'):
            with self.assertRaises(pose.OperatorStop):terminal.handle(key)
        terminal.handle('x')

if __name__=='__main__':unittest.main()
