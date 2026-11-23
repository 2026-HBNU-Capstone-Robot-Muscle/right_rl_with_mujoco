"""Verify complete preserved motion, operator gates, and real health-failure boundaries."""
import tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import leap_measured_grip_hold_20261117 as pose
import quarter_fixed_grip_plan_20261118 as plan
from test_measured_grip_hold_20261117 import FakeBus,Clock,state


class TraceBus(FakeBus):
    def __init__(self,initial):super().__init__(initial);self.command_phases=[]
    def write(self,address,size,values):
        if address==116:self.command_phases.append((self.report.get('phase'),np.asarray(values).copy()))
        return super().write(address,size,values)


class QuarterTests(unittest.TestCase):
    def test_off_start_executes_exact_full_turn_then_fixed_grip_with_gates(self):
        sequence=plan.QuarterFixedGrip();initial=state(0);bus=TraceBus(initial);clock=Clock()
        report=bus.report;report['status']='started';terminal=plan.QuarterTerminal(sequence,report)
        waits={k:0 for k in plan.GATES};checked=set()
        def cancel():
            phase=report.get('phase')
            if phase in plan.GATES and phase not in sequence.accepted:
                waits[phase]+=1
                self.assertEqual(report.get('completed_goals',0),report['phase_gate_indices'][phase])
                if waits[phase]>=5:terminal.handle(' ');checked.add(phase)
            if report.get('holding'):raise pose.OperatorStop()
        rows=sequence.rows(initial['position'])
        with self.assertRaises(pose.OperatorStop):
            pose.run_pose(bus,initial,initial['position'],rows,report,cancel,sequence=sequence,
                          now=clock.now,sleep=clock.sleep,limit=490.)
        full=np.array([q for phase,q in bus.command_phases if phase and phase.startswith('quarter_stage_')])
        self.assertTrue(np.array_equal(full,sequence.goals[:1417]))
        self.assertTrue(np.array_equal(report['last_issued_command'],pose.reference()[2]))
        self.assertEqual(checked,set(plan.GATES));self.assertTrue(all(v>=5 for v in waits.values()))
        self.assertTrue(report['all_torque_off_verified']);self.assertTrue(report['settings_restored'])
        commands=np.array([q for _,q in bus.command_phases]);self.assertLessEqual(np.max(abs(np.diff(commands,axis=0))),2)
    def test_placement_wait_keeps_health_reads_without_new_motion(self):
        sequence=plan.QuarterFixedGrip();initial=state(1)
        initial['position']=sequence.start.copy();initial['goal']=sequence.start.copy()
        bus=TraceBus(initial);clock=Clock();bus.report['status']='started'
        pose.run_pose(bus,initial,sequence.start,sequence.rows(sequence.start),bus.report,lambda:None,
                      sequence=sequence,now=clock.now,sleep=clock.sleep,limit=.8)
        self.assertEqual(bus.report['completed_goals'],1)
        self.assertGreater(len([s for s in bus.report['samples'] if s['phase']=='quarter_place']),3)
        self.assertEqual(len(bus.command_phases),1)
        self.assertTrue(bus.report['all_torque_off_verified'])
    def test_receiving_health_fault_blocks_final_closing(self):
        class FaultBus(TraceBus):
            def snapshot(self):
                value=super().snapshot()
                if self.report.get('phase')=='quarter_receiving_check' and np.any(value['torque']):value['current'][3]=600
                return value
        sequence=plan.QuarterFixedGrip();initial=state(1)
        initial['position']=sequence.start.copy();initial['goal']=sequence.start.copy()
        bus=FaultBus(initial);clock=Clock();bus.report['status']='started'
        terminal=plan.QuarterTerminal(sequence,bus.report)
        def cancel():
            phase=bus.report.get('phase')
            if phase in plan.GATES and phase!='quarter_receiving_check':terminal.handle(' ')
        with self.assertRaisesRegex(RuntimeError,'measured current'):
            pose.run_pose(bus,initial,sequence.start,sequence.rows(sequence.start),bus.report,cancel,
                          sequence=sequence,now=clock.now,sleep=clock.sleep,limit=490.)
        self.assertNotIn('quarter_fixed_grip_close',[p for p,q in bus.command_phases])
        self.assertTrue(bus.report['all_torque_off_verified']);self.assertTrue(bus.report['settings_restored'])
    def test_space_stops_moving_and_q_stops_waiting(self):
        sequence=plan.QuarterFixedGrip();report={'phase':'quarter_stage_7_horizontal_yaw'}
        terminal=plan.QuarterTerminal(sequence,report)
        with self.assertRaises(pose.OperatorStop):terminal.handle(' ')
        self.assertFalse(sequence.accepted)
        report['phase']='quarter_place'
        with self.assertRaises(pose.OperatorStop):terminal.handle('q')
        terminal.handle(' ');self.assertEqual(sequence.accepted,{'quarter_place'})
    def test_changed_candidate_rejected_before_devices(self):
        with tempfile.TemporaryDirectory() as folder:
            altered=Path(folder)/'altered.npz';altered.write_bytes(plan.DATA.read_bytes()+b'changed')
            with patch.object(plan,'DATA',altered):
                with self.assertRaisesRegex(RuntimeError,'candidate changed'):plan.QuarterFixedGrip()

    def test_middle_clears_before_any_thumb_motion_and_other_fingers_stay_fixed(self):
        sequence=plan.QuarterFixedGrip();end=sequence.goals[1416]
        rows=sequence.goals[sequence.phases=='transfer_middle_clear']
        unchanged=[0,1,2,3,4,5,6,7,12,13,14,15]
        self.assertTrue(np.all(rows[:,unchanged]==end[unchanged]))
        self.assertEqual(rows[-1,8:12].tolist(),[3360,2108,2024,1848])
        ready=sequence.goals[np.flatnonzero(sequence.phases=='transfer_middle_clear')[-1]]
        for phase in ('transfer_thumb_prepare','transfer_thumb_push'):
            part=sequence.goals[sequence.phases==phase]
            self.assertTrue(np.all(part[:,4:]==ready[4:]))

    def test_thumb_waypoints_match_taught_geometry_and_grip_is_unchanged(self):
        import json
        sequence=plan.QuarterFixedGrip()
        for phase,name in (('transfer_thumb_prepare','middle_clear'),('transfer_thumb_push','cube_pushed'),
                           ('transfer_thumb_wrist_and_tips','thumb_wrist_clear')):
            ref=json.loads((plan.BASE/(name+'_reference.json')).read_bytes())
            last=sequence.goals[np.flatnonzero(sequence.phases==phase)[-1]]
            self.assertEqual(last[:4].tolist(),ref['actual_ticks'][:4])
        self.assertTrue(np.array_equal(sequence.goals[-1],pose.reference()[2]))

    def test_distal_preparation_overlaps_thumb_only_and_seating_holds_distal_shape(self):
        import json
        sequence=plan.QuarterFixedGrip()
        before=sequence.goals[np.flatnonzero(sequence.phases=='transfer_thumb_push')[-1]]
        overlap=sequence.goals[sequence.phases=='transfer_thumb_wrist_and_tips']
        fixed=[5,6,7,9,10,11,12,13,14,15]
        self.assertTrue(np.all(overlap[:,fixed]==before[fixed]))
        ref=json.loads((plan.BASE/'index_middle_lower_seat_reference.json').read_bytes())
        self.assertEqual(overlap[-1,[4,8]].tolist(),[ref['actual_ticks'][i] for i in (4,8)])
        seat=sequence.goals[sequence.phases=='grip_index_middle_seat']
        fixed=[0,1,2,3,4,8,12,13,14,15]
        self.assertTrue(np.all(seat[:,fixed]==overlap[-1,fixed]))
        self.assertEqual(seat[-1,4:12].tolist(),[3187,2000,2470,2048,3112,2098,2598,1848])

    def test_lower_push_moves_only_index_middle_non_distal_links_before_final_close(self):
        sequence=plan.QuarterFixedGrip()
        before=sequence.goals[np.flatnonzero(sequence.phases=='grip_index_middle_seat')[-1]]
        part=sequence.goals[sequence.phases=='grip_lower_push']
        fixed=[0,1,2,3,4,8,12,13,14,15]
        self.assertTrue(np.all(part[:,fixed]==before[fixed]))
        self.assertEqual(part[-1,[7,11]].tolist(),pose.reference()[2][[7,11]].tolist())
        self.assertEqual(part[-1,[5,6,9,10]].tolist(),[1960,3096,2089,3171])
        close=sequence.goals[sequence.phases=='fixed_grip_close']
        self.assertTrue(np.all(close[:,[7,11]]==part[-1,[7,11]]))


if __name__=='__main__':unittest.main()
