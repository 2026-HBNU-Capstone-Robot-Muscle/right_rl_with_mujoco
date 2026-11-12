"""Reject scope, initial-overlap and missing-evidence regressions without devices."""
import copy,json,unittest,hashlib,sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import single_forward_plan_20261030 as plan
import single_forward_gate_20261030 as gate
class AdmissionTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  with np.load(plan.LOADED) as z:cls.original={k:z[k].copy() for k in z.files}
  cls.fixture=plan.OUT/'software_admission_fixtures';cls.fixture.mkdir(exist_ok=True);cls.cases=[]
  for f in (-15,-10,-5):
   r=json.loads((plan.ROOT/f'right_hand_rotation_20261030/yaw_roundtrip/full_loaded_seated_front_{f}/review.json').read_bytes())
   for c in r['cases']:
    c=copy.deepcopy(c);c['inverse_from_forward_hold_deg']=None;cls.cases.append(c)
 def reject_candidate(self,mutate):
  data={k:v.copy() for k,v in self.original.items()};mutate(data);p=self.fixture/(self._testMethodName+'.npz');np.savez_compressed(p,**data)
  with patch.object(plan,'LOADED',p):
   with self.assertRaises(RuntimeError):plan.validate_candidate()
 def reject_case(self,mutate):
  cases=copy.deepcopy(self.cases);mutate(cases)
  with self.assertRaises(RuntimeError):gate.validate_model_cases(cases)
 def test_exact_completed_forward_is_accepted(self):self.assertEqual(len(plan.validate_candidate()['goals']),2032)
 def test_changed_goal_rejected(self):self.reject_candidate(lambda d:d['goals'].__setitem__((700,0),d['goals'][700,0]+1))
 def test_extra_counterturn_rejected(self):self.reject_candidate(lambda d:d.update(goals=np.vstack([d['goals'],d['goals'][100:108][::-1]]),phases=np.r_[d['phases'],['stage_7_horizontal_yaw']*8]))
 def test_changed_capture_phase_rejected(self):self.reject_candidate(lambda d:d['phases'].__setitem__(646,'stage_9_loaded_open'))
 def test_missing_condition_rejected(self):self.reject_case(lambda c:c.pop())
 def test_duplicate_condition_rejected(self):self.reject_case(lambda c:c.__setitem__(-1,copy.deepcopy(c[0])))
 def test_false_initial_overlap_rejected(self):self.reject_case(lambda c:c[0]['initial_placement_probe'].__setitem__('max_cube_mm',1.6))
 def test_free_upper_endpoint_required(self):self.reject_case(lambda c:c[0].__setitem__('vertical_seating_line_upper_verified_free',False))
 def test_failed_case_cannot_hide_in_all_passed(self):self.reject_case(lambda c:c[0].__setitem__('passed',False))
 def test_self_contact_cannot_be_discarded(self):self.reject_case(lambda c:c[0].__setitem__('max_self_mm',.01))
 def test_lost_retention_rejected(self):self.reject_case(lambda c:c[0].__setitem__('longest_no_hand_contact_seconds',.3))
 def test_inverse_scope_rejected(self):self.reject_case(lambda c:c[0].__setitem__('inverse_from_forward_hold_deg',-42))
 def test_timing_or_capture_change_rejected_before_loading_evidence(self):
  baseline=dict(EMPTY_RELEASE_ONLY=False,CAPTURE_BEFORE_PHASE='stage_5_loaded_grasp',SAMPLE_PERIOD=.075,ON_TIMEOUT=300)
  for key,val in (('EMPTY_RELEASE_ONLY',True),('CAPTURE_BEFORE_PHASE','stage_7_horizontal_yaw'),('SAMPLE_PERIOD',.05),('ON_TIMEOUT',301)):
   p=SimpleNamespace(**dict(baseline,**{key:val}))
   with self.subTest(key=key):
    with self.assertRaises(RuntimeError):gate.make_review(p,{}, {})
 def test_reproduction_review_does_not_certify90_or_inverse(self):
  p=SimpleNamespace(EMPTY_RELEASE_ONLY=False,CAPTURE_BEFORE_PHASE='stage_5_loaded_grasp',SAMPLE_PERIOD=.075,ON_TIMEOUT=300)
  parent=dict(loaded_model_cases=27,loaded_model_all_passed=True,loaded_model_nominal_angle_deg=62,loaded_geometry_cases=201,physical_rotation_verified=False)
  with patch.object(gate.transport,'make_review',return_value=parent),patch.object(gate,'hand_review',return_value={}),patch.object(gate,'cube_models',return_value={}),patch.object(gate,'hashes',return_value={'frozen':'hash'}):
   r=gate.make_review(p,{}, {'frozen':'hash'})
  self.assertFalse(r['physical_exact90_verified']);self.assertFalse(r['inverse90_ready']);self.assertFalse(r['automatic_inverse']);self.assertFalse(r['physical_repeatability_verified'])
 def test_unreviewed_entry_opens_no_devices(self):
  import leap_single_forward_trial_20261030 as entry
  with patch.object(entry,'validate_review',side_effect=RuntimeError('rejected synthetic proof')),patch.object(entry,'SingleForwardBus') as bus,patch.object(entry.runtime,'main') as run:
   self.assertEqual(entry.main(),2);bus.assert_not_called();run.assert_not_called()
if __name__=='__main__':
 suite=unittest.defaultTestLoader.loadTestsFromTestCase(AdmissionTests);result=unittest.TextTestRunner(verbosity=2).run(suite)
 report=dict(status='passed' if result.wasSuccessful() else 'failed',tests_run=result.testsRun,failures=len(result.failures),errors=len(result.errors),devices_opened=False,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),controller_and_model_hashes=plan.hashes())
 (plan.OUT/'software_admission_result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');sys.exit(0 if result.wasSuccessful() else 1)
