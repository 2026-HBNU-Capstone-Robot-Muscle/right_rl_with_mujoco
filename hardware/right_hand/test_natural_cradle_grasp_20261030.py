"""Task evidence, scope, frozen fingers, no-device default and SPACEs."""
import copy,json,sys,unittest,hashlib
from pathlib import Path
from unittest.mock import patch
import numpy as np
import natural_cradle_grasp_plan_20261030 as p
import natural_cradle_grasp_gate_20261030 as g
class NaturalTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.plan=p.NaturalCradleGraspPlan(p.REQUEST);cls.empty=json.loads((p.OUT/'empty_recovery_screen/review.json').read_bytes());cls.valid=g.make_review(cls.plan,cls.empty,p.hashes());cls.fixture=p.OUT/'test_fixtures';cls.fixture.mkdir(exist_ok=True)
 def proof_file(self,value):
  path=self.fixture/(self._testMethodName+'.json');path.write_text(json.dumps(value),encoding='utf-8');return path
 def altered_cube(self,change):
  original=g.json.loads
  def load(data,*args,**kwargs):
   r=original(data,*args,**kwargs)
   if isinstance(r,dict) and r.get('status')=='natural_cradle_free_grasp_hold_release_diagnostic':change(r)
   return r
  return patch.object(g.json,'loads',side_effect=load)
 def test_scope_has_grasp_hold_open_and_no_rotation(self):
  z=p.validate_candidate();self.assertEqual(len(z['goals']),1808);self.assertEqual(list(dict.fromkeys(z['phases'].tolist())),list(p.ALLOWED));self.assertFalse(any('yaw' in s for s in z['phases']))
 def test_candidate_retains_exact_fixed_index_and_ring(self):
  z=p.validate_candidate();np.testing.assert_array_equal(z['goals'][:,p.FIXED],np.tile(z['start'][p.FIXED],(1808,1)))
 def test_every_goal_respects_original_step_and_native_range(self):
  z=p.validate_candidate();lo,hi=p.limits();self.assertTrue(np.all(z['goals']>=lo));self.assertTrue(np.all(z['goals']<=hi));self.assertLessEqual(np.max(abs(np.diff(np.vstack([z['start'],z['goals']]),axis=0))),2)
 def test_new_targets_are_checked_against_current_off_travel_limit(self):
  rows=self.plan.plan(self.plan.source_position());self.assertEqual(len(rows),2352);self.assertLessEqual(np.max(abs(np.array([q for _,q in rows])[:,:4]-self.plan.source_position()[:4])),1300)
 def test_original_timing_and_force_policy(self):
  self.assertEqual(self.plan.SAMPLE_PERIOD,.075);self.assertEqual(self.plan.ON_TIMEOUT,300);self.assertEqual(self.valid['current_ma'],450);self.assertEqual(self.valid['hardware_tracking_stop_rad'],.18);self.assertFalse(self.valid['automatic_goal_or_setting_resend'])
 def test_nominal_contactless_hold_cannot_pass_safety_only(self):
  def change(r):
   for c in r['cases']:
    if c['gain_scale']==1. and c['friction']==.3:c['grasp_hold_observation']['opposing_contact_fraction']['mf_tip']=0.
  with self.altered_cube(change),self.assertRaises(RuntimeError):g.loaded_models()
 def test_unstable_hold_rejected(self):
  def change(r):
   for c in r['cases']:
    if c['gain_scale']==1. and c['friction']==.3:c['grasp_hold_observation']['max_center_shift_mm']=3.
  with self.altered_cube(change),self.assertRaises(RuntimeError):g.loaded_models()
 def test_nonnominal_weak_grasps_are_disclosed_not_certified(self):
  r=g.loaded_models();self.assertEqual(r['cube_safety_cases'],27);self.assertEqual(r['grasp_task_passed_cases'],9);self.assertEqual(len(r['nonnominal_grasp_task_failures']),18);self.assertFalse(r['robust_grasp_verified'])
 def test_missing_condition_rejected(self):
  with self.altered_cube(lambda r:r['cases'].pop()),self.assertRaises(RuntimeError):g.loaded_models()
 def test_duplicate_condition_rejected(self):
  with self.altered_cube(lambda r:r['cases'].__setitem__(1,copy.deepcopy(r['cases'][0]))),self.assertRaises(RuntimeError):g.loaded_models()
 def test_failed_case_rejected_despite_all_passed_flag(self):
  with self.altered_cube(lambda r:r['cases'][0].update(passed=False)),self.assertRaises(RuntimeError):g.loaded_models()
 def test_wrong_model_source_hash_rejected(self):
  with self.altered_cube(lambda r:r.update(model_source_sha256='0'*64)),self.assertRaises(RuntimeError):g.loaded_models()
 def test_wrong_object_seed_hash_rejected(self):
  with self.altered_cube(lambda r:r.update(cube_seed_sha256='0'*64)),self.assertRaises(RuntimeError):g.loaded_models()
 def test_wrong_candidate_hash_rejected(self):
  with self.altered_cube(lambda r:r.update(candidate_sha256='0'*64)),self.assertRaises(RuntimeError):g.loaded_models()
 def test_raw_hand_collision_rejected(self):
  original=g.json.loads
  def load(data,*a,**kw):
   r=original(data,*a,**kw)
   if isinstance(r,dict) and r.get('status')=='pitch_and_transfer_finite_hand_dynamics':r['cases'][0]['raw_max_self_mm']=.1
   return r
  with patch.object(g.json,'loads',side_effect=load),self.assertRaises(RuntimeError):g.loaded_models()
 def test_failed_empty_case_rejected(self):
  empty=copy.deepcopy(self.empty);empty['dynamic_cases'][0]['passed']=False
  with self.assertRaises(RuntimeError):g.make_review(self.plan,empty,p.hashes())
 def test_altered_empty_digest_rejected(self):
  empty=copy.deepcopy(self.empty);empty['prefix_sha256']='0'*64
  with self.assertRaises(RuntimeError):g.make_review(self.plan,empty,p.hashes())
 def test_grasp_does_not_claim_physical_rotation_or_registration(self):
  self.assertEqual(self.valid['rotation_targets'],0);self.assertFalse(self.valid['physical_rotation_verified']);self.assertFalse(self.valid['natural_cradle_models']['body_marker_to_robot_registration_verified']);self.assertFalse(self.valid['elevated_reference_matching_required'])
 def test_changed_rotation_claim_rejected(self):
  r=copy.deepcopy(self.valid);r['physical_rotation_verified']=True
  with self.assertRaises(RuntimeError):g.validate_review(self.plan,self.proof_file(r))
 def test_changed_code_hash_rejected(self):
  path=self.proof_file(self.valid)
  with patch.object(g,'hashes',return_value={'changed':True}),self.assertRaises(RuntimeError):g.validate_review(self.plan,path)
 def test_fixed_grip_and_opening_phase_guards(self):
  np.testing.assert_array_equal(self.plan.frozen_for_phase(p.ALLOWED[0]),p.FIXED);np.testing.assert_array_equal(self.plan.frozen_for_phase(p.ALLOWED[2]),p.FIXED);np.testing.assert_array_equal(self.plan.progress_for_phase(p.ALLOWED[0]),p.ACTIVE);self.assertEqual(len(self.plan.progress_for_phase(p.ALLOWED[1])),0)
 def test_wrong_timing_or_scope_rejected(self):
  for key,value in (('SAMPLE_PERIOD',.05),('ON_TIMEOUT',301),('CAPTURE_BEFORE_PHASE','stage_7_horizontal_yaw'),('EMPTY_RELEASE_ONLY',True)):
   plan=copy.copy(self.plan);setattr(plan,key,value)
   with self.subTest(key=key),self.assertRaises(RuntimeError):g.make_review(plan,self.empty,p.hashes())
 def test_default_entry_opens_no_devices(self):
  import leap_natural_cradle_grasp_20261030 as entry
  with patch.object(entry,'validate_review',return_value=self.valid),patch.object(entry,'NaturalBus') as bus,patch.object(entry,'NaturalCamera') as camera,patch.object(entry.runtime,'run') as run,patch.object(sys,'argv',['natural_grasp']):
   self.assertEqual(entry.main(),0);bus.assert_not_called();camera.assert_not_called();run.assert_not_called()
 def test_camera_keeps_exposure_and_has_no_projected_target_worker(self):
  from leap_natural_cradle_grasp_20261030 import NaturalCamera
  from empty_trial_camera import EmptyTrialCamera
  r={}
  with patch.object(EmptyTrialCamera,'__init__',return_value=None) as init:
   NaturalCamera(self.fixture/'camera',r);self.assertIsNone(init.call_args.kwargs['worker'])
  self.assertEqual(r['camera_exposure_control'],'unchanged');self.assertFalse(r['marker_visibility_is_motion_gate'])
 def test_capture_uses_two_fresh_post_preparation_spaces_without_bus_writes(self):
  from natural_cradle_grasp_capture_20261030 import NaturalCradleCapture
  from held_joint_guard import HeldJointGuard
  command=np.load(p.OPEN_PATH)['open_goal'];report={'samples':[]};clock=[0.]
  class Bus:
   pending=[]
   @staticmethod
   def checked_sample(bus,cmd,r,phase,**kw):
    state={'position':cmd.copy(),'goal':cmd.copy()};r['samples'].append({'phase':phase})
    if kw.get('snapshot_check'):kw['snapshot_check'](state)
    return state,np.zeros(16)
   def write(self,*a,**kw):raise AssertionError('Capture attempted a position write')
  class Camera:
   def capture_named(self,name,*,after):return {'image':name,'camera_read_monotonic':after+.1}
  class Keys:
   arms=0
   def arm(self):self.arms+=1
   def consume_space(self):return True
  def sleep(seconds):clock[0]+=seconds
  keys=Keys();bus=Bus();protocol=NaturalCradleCapture(Camera(),keys,cancel=lambda:None,sleep=sleep,now=lambda:clock[0]);guard=HeldJointGuard(command,command);protocol.place(bus,command,report,guard);protocol.hold_frame(bus,command,report,{'position':command});protocol.finish(bus,command,report)
  self.assertEqual(keys.arms,2);self.assertTrue(report['cube_removal_confirmed']);self.assertFalse(report['elevated_reference_matching_required']);self.assertFalse(report['physical_rotation_verified'])
if __name__=='__main__':
 result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(NaturalTests));report=dict(status='passed' if result.wasSuccessful() else 'failed',tests_run=result.testsRun,failures=len(result.failures),errors=len(result.errors),devices_opened=False,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),controller_hashes=p.hashes());(p.OUT/'software_admission_result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');raise SystemExit(not result.wasSuccessful())
