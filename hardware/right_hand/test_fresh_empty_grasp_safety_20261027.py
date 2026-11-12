"""No-device checks for default behavior, write admission and exact queue order."""
import copy,hashlib,json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import sequence_flip_checked as checked
from fresh_empty_grasp_plan_20261027 import FreshEmptyGraspPlan,REQUEST,PROOF,ROOT,RING_PHASE,ROOT_PHASE
from leap_fresh_empty_grasp_20261027 import FreshEmptyBus
from empty_precise_voltage_20261027 import sample
from fresh_empty_grasp_gate_20261027 import validate_proof,run_screen

class EmptySafety(unittest.TestCase):
    def setUp(self):self.plan=FreshEmptyGraspPlan(REQUEST)
    def bus(self,approved=False,pending=None):
        b=object.__new__(FreshEmptyBus);b.screen_approved=approved;b.pending=[] if pending is None else pending
        return b
    def test_no_proof_prevents_setting_write(self):
        b=self.bus()
        with patch.object(checked.Bus,'write') as write:
            with self.assertRaises(RuntimeError):b.write(102,2,np.full(16,450))
            write.assert_not_called()
    def test_emergency_off_is_available_without_proof(self):
        with patch.object(checked.Bus,'write') as write:self.bus().write(64,1,np.zeros(16,int));write.assert_called_once()
    def test_wrong_goal_rejected_before_write(self):
        q=self.plan.source_position();b=self.bus(True,[q.copy()]);wrong=q.copy();wrong[0]+=1
        with patch.object(checked.Bus,'write') as write:
            with self.assertRaises(ValueError):b.write(116,4,wrong)
            write.assert_not_called();np.testing.assert_array_equal(b.pending[0],q)
    def test_exact_goal_consumed_once_no_resend(self):
        q=self.plan.source_position();b=self.bus(True,[q.copy()])
        with patch.object(checked.Bus,'write') as write:
            b.write(116,4,q,verify=False)
            with self.assertRaises(ValueError):b.write(116,4,q,verify=False)
            write.assert_called_once();self.assertEqual(b.pending,[])
    def test_empty_sampler_rejects_loaded_scope(self):
        b=SimpleNamespace(plan_module=SimpleNamespace(FRESH_EMPTY_GRASP_MODE=True,CAPTURE_PLACEMENT=True))
        with patch('empty_precise_voltage_20261027._healthy') as read:
            with self.assertRaises(RuntimeError):sample(b,np.zeros(16),{},'test')
            read.assert_not_called()
    def test_initial_release_moves_only_reviewed_ring_and_thumb_joints(self):
        a=self.plan.source_position();rows=self.plan.plan(a);ring=[q for p,q in rows if p==RING_PHASE];self.assertEqual(len(ring),40)
        fixed=np.array([i for i in range(16) if i not in (3,12,14)]);self.assertTrue(all(np.array_equal(q[fixed],a[fixed]) for q in ring));np.testing.assert_array_equal(ring[-1][[3,12,14]]-a[[3,12,14]],[16,-80,-80]);self.assertLessEqual(np.max(abs(np.diff(np.vstack([a]+ring),axis=0))),2)
    def test_thumb_clearance_is_completed_before_early_camera_hold(self):
        a=self.plan.source_position();ring=[q for phase,q in self.plan.plan(a) if phase==RING_PHASE];self.assertEqual(int(ring[0][3]-a[3]),2);self.assertEqual(int(ring[7][3]-a[3]),16)
    def test_root_phase_holds_released_ring(self):
        rows=self.plan.plan(self.plan.source_position());ring_end=[q for p,q in rows if p==RING_PHASE][-1];root=[q for p,q in rows if p==ROOT_PHASE];fixed=np.array([i for i in range(16) if i not in (7,11)])
        self.assertTrue(all(np.array_equal(q[fixed],ring_end[fixed]) for q in root))
    def test_root_targets_advance_at_reduced_rates(self):
        rows=self.plan.plan(self.plan.source_position());ring_end=[q for p,q in rows if p==RING_PHASE][-1];root=[q for p,q in rows if p==ROOT_PHASE];values=np.vstack([ring_end]+root);self.assertEqual(self.plan.ROOT_TAIL_GOAL_STEP,1);self.assertLessEqual(int(np.max(abs(np.diff(values[:81],axis=0)))),2);self.assertEqual(int(max(abs(values[80]-ring_end))),120);self.assertLessEqual(int(np.max(abs(np.diff(values[80:],axis=0)))),1);self.assertGreater(len(root),500)
    def test_final_posture_matches_previous_reviewed_endpoint(self):
        old=json.loads(REQUEST.with_name('empty_execution_review_v3.json').read_bytes());np.testing.assert_array_equal(self.plan.plan(self.plan.source_position())[-1][1],old['last_goal_ticks'])
    def test_shared_voltage_and_health_code_unchanged(self):
        old=json.loads(REQUEST.with_name('empty_execution_review_v3.json').read_bytes())
        for name in ('loaded_repeat_voltage_20261026.py','empty_precise_voltage_20261027.py','sequence_flip_checked.py'):
            self.assertEqual(hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),old['code_hashes'][name])
    def test_root_target_blocked_until_actual_ring_is_clear(self):
        rows=self.plan.plan(self.plan.source_position());goal=next(q for phase,q in rows if phase==ROOT_PHASE);b=self.bus(True,[goal.copy()]);b.adopted=True;b.report={'phase':ROOT_PHASE};b.ring_release_actual_threshold=2612;b.last_reviewed_state={'position':self.plan.source_position()}
        with patch.object(checked.Bus,'write') as write:
            with self.assertRaises(RuntimeError):b.write(116,4,goal,verify=False)
            write.assert_not_called();np.testing.assert_array_equal(b.pending[0],goal)
    def test_root_target_requires_a_reviewed_actual_read(self):
        goal=next(q for phase,q in self.plan.plan(self.plan.source_position()) if phase==ROOT_PHASE);b=self.bus(True,[goal.copy()]);b.adopted=True;b.report={'phase':ROOT_PHASE};b.ring_release_actual_threshold=2612
        with patch.object(checked.Bus,'write') as write:
            with self.assertRaises(RuntimeError):b.write(116,4,goal,verify=False)
            write.assert_not_called()
    def test_root_target_blocked_if_ring_tip_joint_did_not_clear(self):
        a=self.plan.source_position();goal=next(q for phase,q in self.plan.plan(a) if phase==ROOT_PHASE);b=self.bus(True,[goal.copy()]);b.adopted=True;b.report={'phase':ROOT_PHASE};b.ring_release_actual_threshold=int(a[14])-10;b.ring_dip_actual_threshold=int(a[12])-10;actual=a.copy();actual[14]-=80;b.last_reviewed_state={'position':actual}
        with patch.object(checked.Bus,'write') as write:
            with self.assertRaises(RuntimeError):b.write(116,4,goal,verify=False)
            write.assert_not_called()
    def test_root_target_allowed_after_both_ring_joints_clear(self):
        a=self.plan.source_position();rows=self.plan.plan(a);goal=next(q for phase,q in rows if phase==ROOT_PHASE);b=self.bus(True,[goal.copy()]);b.adopted=True;b.report={'phase':ROOT_PHASE};b.ring_release_actual_threshold=int(a[14])-10;b.ring_dip_actual_threshold=int(a[12])-10;b.last_reviewed_state={'position':[q for phase,q in rows if phase==RING_PHASE][-1].copy()}
        with patch.object(checked.Bus,'write') as write:b.write(116,4,goal,verify=False);write.assert_called_once()
    def test_fresh_one_tick_shift_keeps_four_tick_screen_available(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            self.assertTrue(Path(directory).resolve().is_relative_to(ROOT.resolve()));path=Path(directory)/'request.json';r=json.loads(REQUEST.read_bytes());r['off_state']['position'][1]+=1;path.write_text(json.dumps(r),encoding='utf-8');plan=FreshEmptyGraspPlan(path);start=plan.source_position()+np.full(16,4);rows=plan.plan(start);values=np.vstack([start]+[q for _,q in rows]);self.assertLessEqual(int(np.max(abs(np.diff(values,axis=0)))),2)
    def test_changed_request_is_rejected(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            self.assertTrue(Path(directory).resolve().is_relative_to(ROOT.resolve()))
            p=Path(directory)/'request.json';p.write_bytes(REQUEST.read_bytes());plan=FreshEmptyGraspPlan(p);r=json.loads(p.read_bytes());r['source_readonly_evidence']='changed';p.write_text(json.dumps(r),encoding='utf-8')
            with self.assertRaises(ValueError):plan.source_position()
    def test_diagnostic_proof_not_execution_permission(self):
        p=REQUEST.with_name('nominal_execution_diagnostic_v1.json')
        with self.assertRaises(RuntimeError):validate_proof(self.plan,p)
    def test_completed_proof_matches_exact_current_plan(self):
        r=validate_proof(self.plan,PROOF);self.assertTrue(r['accepted_for_empty_only']);self.assertFalse(r['loaded_motion_allowed'])
    def test_changed_current_pose_has_its_own_complete_proof(self):
        base=ROOT/'right_hand_rotation_20261026/changed_off_pose_20261027';plan=FreshEmptyGraspPlan(base/'request_v7.json');r=validate_proof(plan,base/'review_v7.json');self.assertTrue(r['accepted_for_empty_only'])
    def test_tampered_proof_cannot_grant_motion(self):
        original=json.loads(PROOF.read_bytes())
        mutations=[lambda r:r['static_screens'].pop(),lambda r:r['last_goal_ticks'].__setitem__(7,1965),lambda r:r['dynamic_sensitivity'][0].__setitem__('native_range_excess_rad',float('nan')),lambda r:r['camera_gap_hold_sensitivity'][0].__setitem__('max_self_after_ring_mm',.01),lambda r:r.__setitem__('loaded_motion_allowed',True)]
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            self.assertTrue(Path(directory).resolve().is_relative_to(ROOT.resolve()));path=Path(directory)/'proof.json'
            for change in mutations:
                r=copy.deepcopy(original);change(r);path.write_text(json.dumps(r),encoding='utf-8')
                with self.assertRaises(RuntimeError):validate_proof(self.plan,path)
    def test_exact_start_reuses_checked_proof_without_simulator(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            self.assertTrue(Path(directory).resolve().is_relative_to(ROOT.resolve()));path=Path(directory)/'proof.json'
            with patch('fresh_empty_grasp_gate_20261027.process_screen') as process:
                r=run_screen(REQUEST,path);self.assertTrue(r['exact_start_cached_proof_used']);process.assert_not_called()
if __name__=='__main__':unittest.main()
