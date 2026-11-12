"""Admission regressions: scope, source binding and current-pose cache limits."""
import copy,json,unittest
from unittest.mock import patch
from supported_inverse_empty_plan_20261029 import SupportedInverseEmptyPlan,OUT,REQUEST,PROOF
import supported_inverse_empty_gate_20261029 as gate

class AdmissionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan=SupportedInverseEmptyPlan(REQUEST);cls.proof=gate.validate_review(cls.plan,PROOF)
        cls.folder=OUT/'guard_test_fixtures';cls.folder.mkdir(exist_ok=True)
    def reject(self,change,name):
        r=copy.deepcopy(self.proof);change(r);path=self.folder/name
        path.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
        with self.assertRaises(RuntimeError):gate.validate_review(self.plan,path)
    def test_cube_scope_cannot_be_promoted(self):
        p=SupportedInverseEmptyPlan(REQUEST);p.EMPTY_RELEASE_ONLY=False
        with self.assertRaises(RuntimeError):gate.make_review(p,self.proof['empty_recovery'],gate.hashes())
    def test_tampered_empty_claim_is_rejected(self):
        self.reject(lambda r:r.update(cube_stays_OUT=False),'scope_tampered.json')
    def test_tampered_route_hash_is_rejected(self):
        self.reject(lambda r:r['new_empty_hand_route'].update(path_sha256='0'*64),'route_tampered.json')
    def test_incomplete_finite_count_is_rejected(self):
        self.reject(lambda r:r['new_empty_hand_route'].update(finite_cases=8),'count_tampered.json')
    def fresh(self,ticks):
        r=json.loads(REQUEST.read_bytes());r['off_state']['position'][0]+=ticks
        path=self.folder/f'fresh_tick{ticks}.json';path.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return path
    def test_one_tick_drift_uses_bound_cache(self):
        source=self.fresh(1);output=self.folder/'cached_tick1_review.json'
        with patch.object(gate,'process_screen',side_effect=AssertionError('unnecessary model rerun')):
            r=gate.run_screen(source,output)
        self.assertTrue(r['cube_stays_OUT']);self.assertTrue(r['review_center_reused'])
        self.assertEqual(r['raw_off_ticks'][0],self.plan.source_position()[0]+1)
    def test_five_tick_drift_requires_a_new_model_review(self):
        source=self.fresh(5);output=self.folder/'uncached_tick5_review.json'
        with patch.object(gate,'process_screen',side_effect=RuntimeError('fresh-prefix-required')) as screen:
            with self.assertRaisesRegex(RuntimeError,'fresh-prefix-required'):gate.run_screen(source,output)
        self.assertEqual(screen.call_count,1)
if __name__=='__main__':unittest.main()
