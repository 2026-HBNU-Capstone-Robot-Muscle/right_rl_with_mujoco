"""Device-free gate, mapping, bounded-goal and release-guard checks."""
import json,tempfile,time,unittest
from pathlib import Path
import numpy as np
from reverse_start_empty_plan_20261025 import ReverseStartPlan,ROOT,TARGET,RING_RELEASE
from reverse_start_empty_gate_20261025 import validate_proof
from leap_reverse_start_empty_20261025 import ReverseStartBus

FIXTURE=ROOT/'right_hand_rotation_20261024/reverse_empty_entry/fixture_v2'

class Checks(unittest.TestCase):
    def setUp(self):
        self.plan=ReverseStartPlan(FIXTURE/'request.json')
        self.proof=json.loads((FIXTURE/'screen.json').read_bytes())
    def reject_mutation(self,change):
        change(self.proof)
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'screen.json';p.write_text(json.dumps(self.proof),encoding='utf-8')
            with self.assertRaises(RuntimeError):validate_proof(self.plan,p)
    def test_complete_real_fixture(self):
        self.assertTrue(validate_proof(self.plan,FIXTURE/'screen.json')['accepted_for_supervised_empty_preparation'])
    def test_no_cube_and_bounded_goals(self):
        raw=self.plan.source_position();rows=self.plan.plan(raw);values=np.vstack([raw]+[q for _,q in rows])
        self.assertFalse(self.plan.CAPTURE_PLACEMENT);self.assertIsNone(self.plan.CAPTURE_BEFORE_PHASE)
        self.assertLessEqual(np.max(abs(np.diff(values,axis=0))),2)
        np.testing.assert_array_equal(values[-1],TARGET)
        limits=json.loads((ROOT/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())
        mask=np.arange(16)!=3
        self.assertTrue(np.all(values[:,mask]>=np.array(limits['lower_ticks'])[mask]))
        self.assertTrue(np.all(values[:,mask]<=np.array(limits['upper_ticks'])[mask]))
        passive=values[:,3]<limits['lower_ticks'][3]
        self.assertTrue(np.all(np.diff(values[passive,3])>=0))
        self.assertLess((len(rows)+20)*.06+2,self.plan.ON_TIMEOUT)
    def test_reject_loaded_claim(self):
        self.reject_mutation(lambda d:d.update(rotation_included=True))
    def test_reject_missing_sensitivity_case(self):
        self.reject_mutation(lambda d:d['static_screens'].pop())
    def test_reject_missing_housing(self):
        self.reject_mutation(lambda d:d['model_manifests'][0]['middle_ring_proxies'].pop())
    def test_reject_unapproved_post_release_contact(self):
        self.reject_mutation(lambda d:d['dynamic_sensitivity'][0].update(max_self_after_release_mm=.01))
    def test_reject_changed_request(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'request.json';p.write_bytes((FIXTURE/'request.json').read_bytes());plan=ReverseStartPlan(p)
            p.write_bytes(p.read_bytes()+b' ')
            with self.assertRaises(ValueError):plan.source_position()
    def test_no_target_advance_without_measured_ring_release(self):
        bus=ReverseStartBus.__new__(ReverseStartBus);bus.approve(self.plan,FIXTURE/'screen.json')
        bus.report={};bus._goal_writes=2+bus._ring_release_writes
        bus.latest_release_state={'position':self.plan.source_position()};bus.latest_release_read_at=time.monotonic()
        with self.assertRaisesRegex(RuntimeError,'Ring release not measured'):
            bus.write(116,4,self.plan.source_position())

if __name__=='__main__':unittest.main()
