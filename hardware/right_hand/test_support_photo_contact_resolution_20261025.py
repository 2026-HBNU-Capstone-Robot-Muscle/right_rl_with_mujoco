import copy,json,tempfile,unittest
from pathlib import Path
from support_photo_contact_resolution_20261025 import classify,metadata,PAIRS,NUMERICAL_ZERO_MM
from support_photo_path_plan_20261025 import FIXTURE,SupportPhotoPathPlan,validate_proof
class ResolutionChecks(unittest.TestCase):
    def test_only_two_initial_seams_at_or_below_resolution_are_zero(self):
        pairs={p:NUMERICAL_ZERO_MM for p in PAIRS};pairs['unrelated|geom']=.001
        self.assertEqual(classify(pairs,initial_release=True),{'unrelated|geom':.001})
    def test_deeper_initial_contact_is_not_hidden(self):
        pairs={p:NUMERICAL_ZERO_MM+.000001 for p in PAIRS};self.assertEqual(classify(pairs,initial_release=True),pairs)
    def test_strict_classification_after_initial_release(self):
        pairs={p:.001 for p in PAIRS};self.assertEqual(classify(pairs,initial_release=False),pairs)
    def test_classifier_does_not_mutate_contact_data_and_declares_forces_preserved(self):
        pairs={p:.005 for p in PAIRS};original=pairs.copy();classify(pairs,initial_release=True);self.assertEqual(pairs,original)
        self.assertTrue(metadata()['contact_forces_preserved']);self.assertFalse(metadata()['physical_contact_calibration_verified'])
    def test_independent_gate_rejects_enlarged_scope_or_discarded_depth(self):
        plan=SupportPhotoPathPlan(FIXTURE/'request.json');proof=json.loads((FIXTURE/'screen.json').read_bytes())
        for kind in ('threshold','depth','pair'):
            wrong=copy.deepcopy(proof)
            if kind=='threshold':wrong['photo_initial_thumb_contact_resolution']['numerical_zero_mm']=.02
            elif kind=='depth':wrong['initial_resolution_discarded_maximums_by_front'][0]['palm_collision_7|th_tip']=.011
            else:wrong['initial_resolution_discarded_maximums_by_front'][0]['unrelated|geom']=.001
            with tempfile.TemporaryDirectory() as folder:
                p=Path(folder)/'proof.json';p.write_text(json.dumps(wrong),encoding='utf-8')
                with self.assertRaisesRegex(RuntimeError,'resolution'):validate_proof(plan,p)
if __name__=='__main__':unittest.main()
