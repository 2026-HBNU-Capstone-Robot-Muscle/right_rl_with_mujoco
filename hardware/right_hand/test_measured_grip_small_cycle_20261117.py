"""Meaningful initialization, sequence retention and fault-boundary checks."""
import copy,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import leap_measured_grip_hold_20261117 as pose
import measured_grip_small_cycle_plan_20261117 as plan
from test_measured_grip_hold_20261117 import FakeBus,Clock,state


class SmallCycleTests(unittest.TestCase):
    def run_sequence(self,torque,bus_factory=FakeBus):
        initial=state(torque);bus=bus_factory(initial);report=bus.report
        report['status']='started';sequence=plan.SmallCycle();clock=Clock()
        start=initial['goal'].copy() if torque else initial['position'].copy()
        rows=sequence.rows(start)
        def cancel():
            if report.get('holding'):raise pose.OperatorStop()
        try:
            pose.run_pose(bus,initial,start,rows,report,cancel,sequence=sequence,
                          now=clock.now,sleep=clock.sleep,limit=100.)
        except pose.OperatorStop:pass
        return bus,sequence
    def test_off_initialization_keeps_both_turns_and_holds(self):
        bus,sequence=self.run_sequence(0)
        issued=np.array([v for address,v in bus.writes if address==116])
        self.assertTrue(any(np.array_equal(v,sequence.grip) for v in issued))
        self.assertTrue(np.array_equal(issued[-1],sequence.goals[-1]))
        self.assertGreater(issued[:,3].max(),sequence.grip[3]+100)
        self.assertLessEqual(np.max(abs(np.diff(issued,axis=0))),2)
        phases=list(dict.fromkeys(s['phase'] for s in bus.report['samples']))
        for phase in ('small_cycle_reference_hold','small_cycle_turn1','small_cycle_hold1','small_cycle_turn2','small_cycle_hold2','small_cycle_hold'):
            self.assertIn(phase,phases)
        self.assertTrue(bus.report['all_torque_off_verified'])
        self.assertTrue(bus.report['settings_restored'])
    def test_on_start_preserves_support_until_final_operator_stop(self):
        bus,sequence=self.run_sequence(1)
        last_goal=max(i for i,(address,_) in enumerate(bus.writes) if address==116)
        first_off=min(i for i,(address,v) in enumerate(bus.writes) if address==64 and not np.any(v))
        self.assertGreater(first_off,last_goal)
        self.assertTrue(np.array_equal(bus.report['last_issued_command'],sequence.goals[-1]))
    def test_second_turn_health_fault_stops_before_its_first_goal(self):
        class FaultBus(FakeBus):
            def snapshot(self):
                value=super().snapshot()
                if self.report.get('phase')=='small_cycle_turn2' and np.any(value['torque']):value['current'][3]=600
                return value
        initial=state(1);bus=FaultBus(initial);bus.report['status']='started'
        sequence=plan.SmallCycle();clock=Clock();rows=sequence.rows(initial['goal'])
        with self.assertRaisesRegex(RuntimeError,'measured current'):
            pose.run_pose(bus,initial,initial['goal'],rows,bus.report,lambda:None,sequence=sequence,
                          now=clock.now,sleep=clock.sleep,limit=100.)
        issued=[v for address,v in bus.writes if address==116]
        self.assertTrue(np.array_equal(issued[-1],sequence.goals[sequence.phases=='hold1'][-1]))
        self.assertTrue(bus.report['all_torque_off_verified'])
        self.assertTrue(bus.report['settings_restored'])
    def test_changed_trajectory_is_rejected_before_device_access(self):
        with tempfile.TemporaryDirectory() as folder:
            altered=Path(folder)/'altered.npz';altered.write_bytes(plan.DATA.read_bytes()+b'changed')
            with patch.object(plan,'DATA',altered):
                with self.assertRaisesRegex(RuntimeError,'hash mismatch'):plan.load_trial()


if __name__=='__main__':unittest.main()
