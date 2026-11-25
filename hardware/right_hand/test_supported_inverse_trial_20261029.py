"""Safety admission regressions for the genuinely operator-placed Cube trial."""
import copy,json,unittest
from supported_inverse_trial_plan_20261029 import SupportedInverseTrialPlan,OUT,REQUEST,PROOF
import supported_inverse_trial_gate_20261029 as gate
class CubeAdmissionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan=SupportedInverseTrialPlan(REQUEST);cls.proof=gate.validate_review(cls.plan,PROOF)
        cls.folder=OUT/'guard_test_fixtures';cls.folder.mkdir(exist_ok=True)
    def reject(self,change,name):
        r=copy.deepcopy(self.proof);change(r);path=self.folder/name
        path.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
        with self.assertRaises(RuntimeError):gate.validate_review(self.plan,path)
    def test_cannot_skip_real_PLACE(self):
        p=SupportedInverseTrialPlan(REQUEST);p.CAPTURE_BEFORE_PHASE=None
        with self.assertRaises(RuntimeError):gate.make_review(p,self.proof['empty_recovery'],gate.hashes())
    def test_forged_actual_EMPTY_evidence_rejected(self):
        self.reject(lambda r:r['actual_empty_evidence'].update(sha256='0'*64),'physical_tampered.json')
    def test_missing_cube_condition_rejected(self):
        self.reject(lambda r:r['supported_inverse_cube_models'].update(cases=26),'cube_tampered.json')
    def test_success_cannot_be_preclaimed(self):
        self.reject(lambda r:r.update(physical_inverse_90_verified=True),'claim_tampered.json')
if __name__=='__main__':unittest.main()
