"""Test loaded grip adoption, staged handover, exact turn, and fault stops."""
import tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import leap_measured_grip_hold_20261117 as pose
import grip_to_rotation_plan_20261118 as plan
from test_measured_grip_hold_20261117 import FakeBus,Clock,state

class TraceBus(FakeBus):
    def __init__(self,s):super().__init__(s);self.goals=[]
    def write(self,address,size,values):
        if address==116:self.goals.append((self.report.get('phase'),np.asarray(values).copy()))
        return super().write(address,size,values)

class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import json
        cls.temp=tempfile.TemporaryDirectory();fixture=Path(cls.temp.name)/'manifest.json'
        cls.production_manifest=plan.MANIFEST
        manifest=json.loads(plan.MANIFEST.read_bytes());manifest['operator_gated_observation_enabled']=True
        fixture.write_text(json.dumps(manifest),encoding='utf-8')
        cls.manifest_patch=patch.object(plan,'MANIFEST',fixture);cls.manifest_patch.start()

    @classmethod
    def tearDownClass(cls):
        cls.manifest_patch.stop();cls.temp.cleanup()

    def test_loaded_grip_stays_ON_through_full_turn_until_operator_end(self):
        seq=plan.GripToRotation();initial=state(1);bus=TraceBus(initial);clock=Clock()
        bus.report['status']='started';terminal=plan.GripTurnTerminal(seq,bus.report);waits={p:0 for p in plan.GATES}
        def cancel():
            phase=bus.report.get('phase')
            if phase in plan.GATES and phase not in seq.accepted:
                waits[phase]+=1
                if waits[phase]>=4:terminal.handle(' ')
            if bus.report.get('holding'):raise pose.OperatorStop()
        with self.assertRaises(pose.OperatorStop):
            pose.run_pose(bus,initial,initial['goal'],seq.rows(initial['goal']),bus.report,cancel,
                sequence=seq,now=clock.now,sleep=clock.sleep,limit=490.)
        executed=np.array([q for phase,q in bus.goals if phase and phase.startswith('grip_turn_preserved_')])
        self.assertTrue(np.array_equal(executed,seq.goals[seq.preparation:]))
        self.assertTrue(all(v>=4 for v in waits.values()))
        last_goal=max(i for i,(a,_) in enumerate(bus.writes) if a==116)
        self.assertTrue(all(i>last_goal for i,(a,q) in enumerate(bus.writes) if a==64 and not np.any(q)))
        self.assertTrue(bus.report['settings_restored']);self.assertTrue(bus.report['all_torque_off_verified'])

    def test_support_targets_frozen_until_their_handover_phase(self):
        seq=plan.GripToRotation();g=seq.start
        clear=seq.goals[seq.phases=='thumb_clear_for_turn']
        self.assertTrue(np.all(clear[:,4:]==g[4:]))
        transfer=seq.goals[seq.phases=='measured_thumbward_transfer'];fixed=[0,1,2,3,12,13,14,15]
        self.assertTrue(np.all(transfer[:,fixed]==clear[-1,fixed]))
        pushed=seq.goals[seq.phases=='ring_push_thumbward']
        self.assertTrue(np.all(pushed[:,:12]==transfer[-1,:12]))
        released=seq.goals[seq.phases=='release_index'];fixed=[0,1,2,3,8,9,10,11,12,13,14,15]
        self.assertTrue(np.all(released[:,fixed]==pushed[-1,fixed]))
        caught=seq.goals[seq.phases=='thumb_to_turn_catch']
        self.assertTrue(np.all(caught[:,4:]==released[-1,4:]))
        ready=seq.goals[seq.phases=='middle_ring_to_turn_ready']
        self.assertTrue(np.all(ready[:,:8]==caught[-1,:8]))

    def test_preparation_and_turn_stay_inside_unchanged_two_tick_envelope(self):
        seq=plan.GripToRotation();whole=np.vstack([seq.start,seq.goals])
        self.assertLessEqual(np.max(abs(np.diff(whole,axis=0))),2)
        _,high,low=pose.bounds();self.assertTrue(np.all(whole>=low));self.assertTrue(np.all(whole<=high))
        self.assertEqual(len(seq.goals)-seq.preparation,772)

    def test_thumb_clears_to_recorded_waypoint_before_turn_preparation(self):
        import json
        seq=plan.GripToRotation();ref=json.loads((plan.ROOT/'right_hand_rotation_20261118/quarter_turn_fixed_grip/thumb_wrist_clear_reference.json').read_bytes())
        rows=seq.goals[seq.phases=='thumb_clear_for_turn']
        self.assertEqual(rows[-1,:4].tolist(),ref['actual_ticks'][:4])
        self.assertLess(np.flatnonzero(seq.phases=='thumb_clear_for_turn')[-1],np.flatnonzero(seq.phases=='thumb_to_turn_catch')[0])

    def test_transfer_uses_live_index_middle_only_with_inward_corrections(self):
        import json
        seq=plan.GripToRotation()
        ref=json.loads((plan.ROOT/'receiving_waypoint_readonly_20261118_085240_074537/result.json').read_bytes())
        q=np.array(ref['snapshots'][-1]['position']);_,high,low=pose.bounds()
        expected=q[4:12].copy()
        for j,i in enumerate(range(4,12)):
            if expected[j]<low[i] or expected[j]>high[i]:expected[j]=int(np.clip(expected[j],np.ceil(low[i])+4,high[i]))
        self.assertEqual(seq.goals[seq.phases=='measured_thumbward_transfer'][-1,4:12].tolist(),expected.tolist())
        rows=seq.rows(seq.start);gate=seq.labels.index('grip_turn_transfer_check')
        self.assertFalse(seq.ready(gate))
        self.assertTrue(np.array_equal(rows[gate],rows[gate-1]))

    def test_fault_at_turn_ready_blocks_rotation_and_restores_OFF(self):
        class Fault(TraceBus):
            def snapshot(self):
                s=super().snapshot()
                if self.report.get('phase')=='grip_turn_ready_check' and np.any(s['torque']):s['current'][0]=600
                return s
        seq=plan.GripToRotation();initial=state(1);bus=Fault(initial);clock=Clock();bus.report['status']='started'
        terminal=plan.GripTurnTerminal(seq,bus.report)
        def cancel():
            if bus.report.get('phase') in plan.GATES:terminal.handle(' ')
        with self.assertRaisesRegex(RuntimeError,'measured current'):
            pose.run_pose(bus,initial,initial['goal'],seq.rows(initial['goal']),bus.report,cancel,
                sequence=seq,now=clock.now,sleep=clock.sleep,limit=490.)
        self.assertFalse(any(p and p.startswith('grip_turn_preserved_') for p,q in bus.goals))
        self.assertTrue(bus.report['all_torque_off_verified']);self.assertTrue(bus.report['settings_restored'])

    def test_SPACE_only_advances_confirmed_gate_and_stops_motion(self):
        seq=plan.GripToRotation();r={'phase':'grip_turn_thumb_to_turn_ready'};t=plan.GripTurnTerminal(seq,r)
        with self.assertRaises(pose.OperatorStop):t.handle(' ')
        r['phase']='grip_turn_grasp_check';t.handle(' ');self.assertIn(r['phase'],seq.accepted)
        with self.assertRaises(pose.OperatorStop):t.handle('q')

    def test_tampered_candidate_rejected_before_device_open(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'candidate.npz';p.write_bytes(plan.DATA.read_bytes()+b'altered')
            with patch.object(plan,'DATA',p):
                with self.assertRaisesRegex(RuntimeError,'candidate changed'):plan.GripToRotation()

    def test_production_scope_allows_gated_turn_and_keeps_transfer_only_limited(self):
        with patch.object(plan,'MANIFEST',self.production_manifest):
            full=plan.GripToRotation()
            self.assertEqual(len(full.goals)-full.preparation,772)
            seq=plan.GripToRotation(transfer_only=True)
        initial=state(1);bus=TraceBus(initial);clock=Clock();bus.report['status']='started'
        terminal=plan.GripTurnTerminal(seq,bus.report)
        def cancel():
            if bus.report.get('phase') in plan.GATES:terminal.handle(' ')
            if bus.report.get('holding'):raise pose.OperatorStop()
        with self.assertRaises(pose.OperatorStop):
            pose.run_pose(bus,initial,initial['goal'],seq.rows(initial['goal']),bus.report,cancel,
                sequence=seq,now=clock.now,sleep=clock.sleep,limit=490.)
        self.assertTrue(seq.transfer_only)
        self.assertFalse(any('preserved_' in str(p) or 'release_index' in str(p) or 'ring_push' in str(p) for p,q in bus.goals))
        expected=seq.goals[-1]
        self.assertTrue(np.array_equal(bus.goals[-1][1],expected))
        self.assertEqual(bus.report['phase'],'grip_turn_transfer_hold')
        self.assertEqual(bus.report['preserved_turn_goals'],0)
        self.assertTrue(bus.report['all_torque_off_verified']);self.assertTrue(bus.report['settings_restored'])

    def test_observed_transfer_prefix_unchanged_before_full_turn(self):
        import json,hashlib
        seq=plan.GripToRotation()
        source=plan.ROOT/'grip_to_rotation_trial_20261118_090301_661596/result.json'
        receipt=json.loads(source.read_bytes())
        self.assertEqual(receipt['source_candidate_sha256'],seq.manifest['observed_reference_candidate_sha256'])
        self.assertEqual(receipt['completed_goals'],receipt['planned_goals'])
        self.assertTrue(receipt['holding'])
        count=seq.manifest['observed_prefix_goal_count']
        digest=hashlib.sha256(np.asarray(seq.goals[:count],dtype='<i4').tobytes()).hexdigest()
        self.assertEqual(digest,seq.manifest['observed_prefix_goals_sha256'])
        self.assertEqual(str(seq.phases[count-1]),'measured_thumbward_transfer')

    def test_ring_push_uses_recorded_geometry_then_holds_until_catch(self):
        import json
        seq=plan.GripToRotation();raw=json.loads((plan.ROOT/'receiving_waypoint_readonly_20261118_085240_074537/result.json').read_bytes())
        _,high,low=pose.bounds();expected=np.clip(raw['snapshots'][-1]['position'][12:16],np.ceil(low[12:16]),high[12:16]).astype(int)
        pushed=seq.goals[seq.phases=='ring_push_thumbward']
        self.assertEqual(pushed[-1,12:16].tolist(),expected.tolist())
        self.assertFalse(np.array_equal(expected,seq.start[12:16]))
        for name in ('release_index','thumb_to_turn_catch'):
            self.assertTrue(np.all(seq.goals[seq.phases==name,12:16]==expected))
        seq.rows(seq.start)
        catch=seq.labels.index('grip_turn_catch_check')
        release=seq.labels.index('grip_turn_middle_ring_to_turn_ready')
        self.assertLess(catch,release);self.assertFalse(seq.ready(catch))

    def test_fault_at_ring_push_blocks_handover_and_turn(self):
        class Fault(TraceBus):
            def snapshot(self):
                s=super().snapshot()
                if self.report.get('phase')=='grip_turn_ring_push_check' and np.any(s['torque']):s['current'][12]=600
                return s
        seq=plan.GripToRotation();initial=state(1);bus=Fault(initial);clock=Clock();bus.report['status']='started'
        terminal=plan.GripTurnTerminal(seq,bus.report)
        def cancel():
            if bus.report.get('phase') in plan.GATES:terminal.handle(' ')
        with self.assertRaisesRegex(RuntimeError,'measured current'):
            pose.run_pose(bus,initial,initial['goal'],seq.rows(initial['goal']),bus.report,cancel,
                sequence=seq,now=clock.now,sleep=clock.sleep,limit=490.)
        self.assertFalse(any(p and ('preserved_' in p or p=='grip_turn_thumb_to_turn_catch') for p,q in bus.goals))
        self.assertTrue(bus.report['all_torque_off_verified']);self.assertTrue(bus.report['settings_restored'])

if __name__=='__main__':unittest.main()
