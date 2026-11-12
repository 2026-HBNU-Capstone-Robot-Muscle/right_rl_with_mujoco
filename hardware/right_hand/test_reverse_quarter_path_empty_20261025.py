"""Device-free checks for the supervised full EMPTY reference path."""
import json,tempfile,time,unittest,subprocess,sys
from pathlib import Path
from unittest.mock import patch
import numpy as np
from reverse_quarter_path_empty_plan_20261025 import ReverseQuarterEmptyPlan,ROOT,motion_goals,START_HOLD,TURN_PHASE,END_HOLD
from reverse_quarter_path_empty_gate_20261025 import validate_proof
from leap_reverse_quarter_path_empty_20261025 import QuarterEmptyBus
from leap_book_cube_grasp_trial import BookGraspBus
FIXTURE=ROOT/'right_hand_rotation_20261024/reverse_quarter_path_empty/fixture_v1'

class Checks(unittest.TestCase):
    def setUp(self):
        self.plan=ReverseQuarterEmptyPlan(FIXTURE/'request.json');self.proof=json.loads((FIXTURE/'screen.json').read_bytes())
    def rejected(self,mutation):
        mutation(self.proof)
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'proof.json';p.write_text(json.dumps(self.proof),encoding='utf-8')
            with self.assertRaises(RuntimeError):validate_proof(self.plan,p)
    def test_complete_real_input_fixture(self):
        self.assertTrue(validate_proof(self.plan,FIXTURE/'screen.json')['full_quarter_reference_path'])
    def test_whole_motion_and_holds_have_no_cube_checkpoint(self):
        rows=self.plan.plan(self.plan.source_position());motion=np.array([q for p,q in rows if p==TURN_PHASE])
        np.testing.assert_array_equal(motion,motion_goals()[1:])
        self.assertFalse(self.plan.CAPTURE_PLACEMENT);self.assertIsNone(self.plan.CAPTURE_BEFORE_PHASE)
        self.assertEqual(sum(p==START_HOLD for p,q in rows),40)
        for p,q in rows[-20:]:
            self.assertEqual(p,END_HOLD);np.testing.assert_array_equal(q,motion[-1])
        values=np.vstack([self.plan.source_position()]+[q for p,q in rows])
        self.assertLessEqual(abs(np.diff(values,axis=0)).max(),2)
        limits=json.loads((ROOT/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())
        self.assertTrue(np.all(values>=limits['lower_ticks']));self.assertTrue(np.all(values<=limits['upper_ticks']))
        self.assertLess((len(rows)+20)*.06+2,self.plan.ON_TIMEOUT)
    def test_reject_removed_motion_goal(self):
        original=self.plan.plan
        def changed(raw):
            rows=original(raw);rows.pop(next(i for i,(p,q) in enumerate(rows) if p==TURN_PHASE));return rows
        self.plan.plan=changed
        with self.assertRaisesRegex(RuntimeError,'motion/hold mismatch'):validate_proof(self.plan,FIXTURE/'screen.json')
    def test_reject_placement_claim(self):self.rejected(lambda r:r.update(placement_checkpoint_required=True))
    def test_reject_loaded_rotation_claim(self):self.rejected(lambda r:r.update(rotation_included=True))
    def test_reject_missing_static_case(self):self.rejected(lambda r:r['static_screens'].pop())
    def test_reject_missing_native_case(self):self.rejected(lambda r:r['dynamic_sensitivity'].pop())
    def test_reject_missing_housing_geometry(self):self.rejected(lambda r:r['model_manifests'][0]['middle_ring_proxies'].pop())
    def test_reject_new_self_contact(self):self.rejected(lambda r:r['dynamic_sensitivity'][0].update(max_self_after_release_mm=.001))
    def test_reject_extended_support_deadline(self):self.rejected(lambda r:r['dynamic_sensitivity'][0].update(support_escape_deadline=20.))
    def test_reject_modified_request(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'request.json';p.write_bytes((FIXTURE/'request.json').read_bytes());plan=ReverseQuarterEmptyPlan(p)
            p.write_bytes(p.read_bytes()+b' ')
            with self.assertRaises(ValueError):plan.source_position()
    def test_no_advance_before_fresh_measured_base_release(self):
        bus=QuarterEmptyBus.__new__(QuarterEmptyBus);bus.approve(self.plan,FIXTURE/'screen.json')
        bus.report={};bus._goal_writes=2+bus._base_release_writes
        bus.latest_release_state={'position':self.plan.source_position()};bus.latest_release_read_at=time.monotonic()
        with patch.object(BookGraspBus,'write') as write:
            with self.assertRaisesRegex(RuntimeError,'release not measured'):bus.write(116,4,motion_goals()[0])
            write.assert_not_called()
            bus.latest_release_state['position'][3]=bus._base_release_threshold;bus.latest_release_read_at=time.monotonic()-1
            with self.assertRaisesRegex(RuntimeError,'No fresh'):bus.write(116,4,motion_goals()[0])
            write.assert_not_called()
    def test_default_is_device_free(self):
        r=subprocess.run([sys.executable,'-X','utf8','-B',str(ROOT/'leap_reverse_quarter_path_empty_20261025.py')],capture_output=True,text=True,encoding='utf-8',timeout=5)
        self.assertEqual(r.returncode,0);self.assertIn('CHECK ONLY: no camera, motor port, or simulator process opened.',r.stdout)

if __name__=='__main__':unittest.main()
