import contextlib,io,json,sys,unittest
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch
import numpy as np
import raised_support_capture_plan_20261104 as plan
import raised_support_capture_gate_20261104 as gate
class CaptureTests(unittest.TestCase):
 def setUp(self):self.p=plan.RaisedSupportCapturePlan(plan.REQUEST);self.proof=json.loads(plan.PROOF.read_bytes())
 def test_every_post_placement_goal_is_identical(self):
  rows=self.p.plan(self.p.source_position());after=[q for phase,q in rows if phase not in plan.PREFIX_PHASES];self.assertEqual(len(after),27);self.assertTrue(np.all(np.asarray(after)==plan.validate_candidate()['start']));self.assertEqual(set(p for p,q in rows if p not in plan.PREFIX_PHASES),{'stage_5_loaded_grasp'})
 def test_identical_prefix_really_matches_existing201_and9(self):
  data=np.load(plan.TURN_OUT/'empty_recovery_screen/prefix_path.npz');rows=[(p,q) for p,q in self.p.plan(self.p.source_position()) if p in plan.PREFIX_PHASES];np.testing.assert_array_equal(data['goals'],[q for p,q in rows]);np.testing.assert_array_equal(data['phases'],[p for p,q in rows]);self.assertEqual(len(self.proof['empty_recovery']['static_cases']),201);self.assertEqual(len(self.proof['empty_recovery']['dynamic_cases']),9)
 def test_no_rotation_or_grasp_claim(self):
  r=gate.validate_review(self.p,plan.PROOF);self.assertEqual(r['rotation_targets'],0);self.assertEqual(r['grasp_targets'],0);self.assertEqual(r['post_placement_angle_changes'],0);self.assertFalse(r['raised_turn_candidate_admitted']);self.assertFalse(r['physical_rotation_verified'])
 def test_failed_empty_case_is_rejected(self):
  em=deepcopy(self.proof['empty_recovery']);em['static_cases'][0]['passed']=False
  with self.assertRaises(RuntimeError):gate.make_review(self.p,em,plan.hashes())
 def test_loaded_angle_change_is_rejected_even_if_one_tick(self):
  original=self.p.plan;rows=original(self.p.source_position());phase,q=rows[-1];rows[-1]=(phase,q.copy());rows[-1][1][0]+=1
  with patch.object(self.p,'plan',return_value=rows),self.assertRaises(RuntimeError):gate.make_review(self.p,self.proof['empty_recovery'],plan.hashes())
 def test_wrong_start_is_not_reused(self):
  em=deepcopy(self.proof['empty_recovery']);em['start_ticks'][14]+=1
  with self.assertRaises(RuntimeError):gate.make_review(self.p,em,plan.hashes())
 def test_bad_scope_in_saved_proof_is_rejected(self):
  original=Path.read_bytes;bad=deepcopy(self.proof);bad['physical_rotation_verified']=True
  def read(path):return json.dumps(bad).encode() if path==plan.PROOF else original(path)
  with patch.object(Path,'read_bytes',read),self.assertRaises(RuntimeError):gate.validate_review(self.p,plan.PROOF)
 def test_default_main_cannot_open_motor_bus_or_camera(self):
  import leap_raised_support_capture_20261104 as app
  with patch.object(sys,'argv',['leap_raised_support_capture_20261104.py']),patch.object(app.CaptureBus,'__init__',side_effect=AssertionError('No motor bus in default mode')) as bus,patch.object(app.NaturalCamera,'__init__',side_effect=AssertionError('No camera in default mode')) as camera,contextlib.redirect_stdout(io.StringIO()):
   code=app.main()
  self.assertEqual(code,0);self.assertEqual(bus.call_count,0);self.assertEqual(camera.call_count,0)
if __name__=='__main__':
 stream=io.StringIO();result=unittest.TextTestRunner(stream=stream,verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(CaptureTests));report=dict(tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),all_passed=result.wasSuccessful(),motor_commands=0);(plan.OUT/'software_verification.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(stream.getvalue());print(json.dumps(report));raise SystemExit(0 if result.wasSuccessful() else 1)
