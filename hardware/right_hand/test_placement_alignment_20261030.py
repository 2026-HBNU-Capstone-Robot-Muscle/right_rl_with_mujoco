"""Scope, write fence, source binding and default-device isolation tests."""
import copy,json,hashlib,sys,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import placement_alignment_plan_20261030 as plan
import placement_alignment_gate_20261030 as gate
from leap_placement_alignment_20261030 import AlignmentBus
class AlignmentTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.p=plan.PlacementAlignmentPlan(plan.REQUEST);cls.empty=json.loads((plan.OUT/'empty_recovery_screen/review.json').read_bytes());cls.fixture=plan.OUT/'software_fixtures';cls.fixture.mkdir(exist_ok=True)
  cls.valid=gate.make_review(cls.p,cls.empty,plan.hashes())
 def fixture_review(self,r):
  p=self.fixture/(self._testMethodName+'.json');p.write_text(json.dumps(r),encoding='utf-8');return p
 def test_known_empty_prefix_only(self):
  rows=plan.validate_scope(self.p,self.p.source_position());self.assertGreater(len(rows),0);self.assertTrue(all(p in plan.PREFIX_PHASES for p,q in rows));self.assertLessEqual(np.max(abs(np.diff(np.vstack([self.p.source_position()]+[q for p,q in rows]),axis=0))),2)
 def test_injected_grasp_is_rejected(self):
  rows=self.p.plan(self.p.source_position());bad=copy.copy(self.p);bad.plan=lambda raw:rows+[('stage_5_loaded_grasp',rows[-1][1])]
  with self.assertRaises(RuntimeError):plan.validate_scope(bad,bad.source_position())
 def test_missing_open_endpoint_rejected(self):
  rows=self.p.plan(self.p.source_position());bad=copy.copy(self.p);bad.plan=lambda raw:[(phase,q+1) for phase,q in rows]
  with self.assertRaises(RuntimeError):plan.validate_scope(bad,bad.source_position())
 def test_loaded_or_timing_scope_rejected(self):
  for key,value in (('CAPTURE_PLACEMENT',False),('CAPTURE_BEFORE_PHASE','stage_5_loaded_grasp'),('EMPTY_RELEASE_ONLY',True),('SAMPLE_PERIOD',.05),('ON_TIMEOUT',301)):
   bad=copy.copy(self.p);setattr(bad,key,value)
   with self.subTest(key=key),self.assertRaises(RuntimeError):gate.make_review(bad,self.empty,plan.hashes())
 def test_goal_write_fence_precedes_transport(self):
  bus=object.__new__(AlignmentBus);bus.report={'placement_goals_locked':True}
  with patch.object(gate.transport,'make_review'):
   with patch('leap_four_reply_yaw_20261029.FourReplyBus.write') as parent:
    with self.assertRaises(RuntimeError):bus.write(116,4,np.zeros(16,dtype=int))
    parent.assert_not_called()
 def test_torque_off_not_blocked_by_goal_fence(self):
  bus=object.__new__(AlignmentBus);bus.report={'placement_goals_locked':True}
  with patch('leap_four_reply_yaw_20261029.FourReplyBus.write',return_value='OFF') as parent:
   self.assertEqual(bus.write(64,1,np.zeros(16,dtype=int)),'OFF');parent.assert_called_once()
 def test_failed_model_cannot_hide_in_all_passed(self):
  empty=copy.deepcopy(self.empty);empty['dynamic_cases'][0]['passed']=False
  with self.assertRaises(RuntimeError):gate.make_review(self.p,empty,plan.hashes())
 def test_missing_static_case_rejected(self):
  empty=copy.deepcopy(self.empty);empty['static_cases'].pop()
  with self.assertRaises(RuntimeError):gate.make_review(self.p,empty,plan.hashes())
 def test_new_overlap_rejected(self):
  empty=copy.deepcopy(self.empty);empty['dynamic_cases'][0]['max_self_mm']=.01
  with self.assertRaises(RuntimeError):gate.make_review(self.p,empty,plan.hashes())
 def test_changed_prefix_rejected(self):
  empty=copy.deepcopy(self.empty);empty['prefix_sha256']='0'*64
  with self.assertRaises(RuntimeError):gate.make_review(self.p,empty,plan.hashes())
 def test_valid_photo_only_scope_not_rotation_claim(self):
  r=gate.validate_review(self.p,self.fixture_review(self.valid));self.assertFalse(r['loaded_motion_authorized']);self.assertEqual(r['post_placement_goal_writes'],0);self.assertFalse(r['physical_rotation_verified'])
 def test_edited_claim_rejected(self):
  r=copy.deepcopy(self.valid);r['rotation_targets']=1
  with self.assertRaises(RuntimeError):gate.validate_review(self.p,self.fixture_review(r))
 def test_reference_hash_change_rejected(self):
  original=gate.sha
  with patch.object(gate,'sha',side_effect=lambda p:'0'*64 if Path(p)==plan.REFERENCE_IMAGE else original(p)):
   with self.assertRaises(RuntimeError):gate.validate_review(self.p,self.fixture_review(self.valid))
 def test_default_entry_opens_no_devices(self):
  import leap_placement_alignment_20261030 as entry
  with patch.object(entry,'validate_review',return_value=self.valid),patch.object(entry,'AlignmentBus') as bus,patch.object(entry,'AlignmentCamera') as camera,patch.object(entry.runtime,'run') as run,patch.object(sys,'argv',['placement_alignment']):
   self.assertEqual(entry.main(),0);bus.assert_not_called();camera.assert_not_called();run.assert_not_called()
 def test_display_does_not_change_raw_camera_frame(self):
  import cv2 as cv
  from placement_alignment_camera_20261030 import comparison_frame
  frame=np.zeros((720,1280,3),np.uint8);ref=np.full_like(frame,255);saved=frame.copy();display=comparison_frame(cv,frame,ref,'PLACE PHOTO ONLY');self.assertEqual(display.shape,(440,1280,3));np.testing.assert_array_equal(frame,saved);self.assertTrue(np.any(display[:,640:]))
 def test_reference_display_matches_bound_photo(self):
  from placement_alignment_camera_20261030 import REFERENCE
  self.assertEqual(REFERENCE,plan.REFERENCE_IMAGE)
 def test_full_capture_protocol_does_not_write_goals(self):
  from placement_alignment_capture_20261030 import capture
  command=self.p.plan(self.p.source_position())[-1][1];report={'samples':[],'write_events':[]};clock=[0.]
  class Bus:
   pending=[]
   def write(self,*a,**k):raise AssertionError('Capture attempted a bus write')
   @staticmethod
   def checked_sample(bus,cmd,r,phase,**kw):
    state={'position':cmd.copy(),'goal':cmd.copy()};r['samples'].append({'phase':phase})
    if kw.get('snapshot_check'):kw['snapshot_check'](state)
    return state,np.zeros(16)
  class Camera:
   photos=[]
   def capture_named(self,name,*,after):self.photos.append(name);return {'image':name,'camera_read_monotonic':after+.1}
  class Keys:
   arms=0
   def arm(self):self.arms+=1
   def consume_space(self):return True
  def sleep(t):clock[0]+=t
  keys=Keys();camera=Camera();capture(Bus(),command,report,camera,keys,cancel=lambda:None,sleep=sleep,now=lambda:clock[0]);self.assertTrue(report['cube_removal_confirmed']);self.assertTrue(report['placement_goals_locked']);self.assertEqual(report['post_placement_goal_writes'],0);self.assertEqual(keys.arms,2);self.assertEqual(camera.photos,['prepared_empty.jpg','cube_placed.jpg'])
 def test_reference_wireframe_agrees_with_saved_cube_edges(self):
  import cv2 as cv
  from placement_reference_overlay_20261030 import load_model,estimate
  model=load_model();im=cv.imdecode(np.frombuffer(plan.REFERENCE_IMAGE.read_bytes(),np.uint8),cv.IMREAD_COLOR);m=estimate(im,model)
  self.assertLess(m['estimated_distance_mm'],1.);expected=np.array([[547,202],[819,254],[517,429],[757,480]])
  self.assertLess(np.max(np.linalg.norm(np.array(m['target_points'])[[1,3,5,7]]-expected,axis=1)),8.)
 def test_current_low_placement_not_reported_as_matching(self):
  import cv2 as cv
  from placement_reference_overlay_20261030 import load_model,estimate
  path=plan.ROOT/'placement_alignment_capture_20261030_092639_869097/cube_placed.jpg';im=cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR);m=estimate(im,load_model())
  self.assertGreater(m['estimated_distance_mm'],10.);self.assertLess(m['estimated_height_delta_mm'],-5.);self.assertNotIn(5,m['decoded_ids']);self.assertTrue(m['green_is_colour_grid_not_inferred_marker'])
 def test_guide_does_not_invent_hidden_palm(self):
  import placement_reference_overlay_20261030 as overlay
  with patch.object(overlay,'decoded',return_value={}):m=overlay.estimate(np.zeros((720,1280,3),np.uint8),overlay.load_model())
  self.assertFalse(m['target_visible']);self.assertFalse(m['metric_available'])
 def test_overlay_changes_only_display_copy(self):
  import placement_reference_overlay_20261030 as overlay
  frame=np.zeros((720,1280,3),np.uint8);saved=frame.copy();m={'target_visible':True,'target_points':[[200+i*10,200+i*20] for i in range(8)]};view=overlay.draw(frame,m)
  np.testing.assert_array_equal(frame,saved);self.assertTrue(np.any(view));self.assertFalse(np.shares_memory(frame,view))
 def test_worker_renders_overlay_after_draining_instruction_queue(self):
  import cv2 as cv,queue
  import camera_stream as stream
  import placement_alignment_camera_20261030 as camera
  path=plan.ROOT/'placement_alignment_capture_20261030_092639_869097/cube_placed.jpg';raw=cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR);saved=raw.copy();updates=queue.Queue();updates.put('PLACE YELLOW TARGET');shown=[];original=stream.render_preview
  def fake_worker(send,stop,preview):
   instruction,stopped=stream.render_preview(cv,raw,preview,'old instruction');self.assertEqual(instruction,'PLACE YELLOW TARGET');self.assertFalse(stopped)
  with patch.object(stream,'camera_worker',side_effect=fake_worker),patch.object(cv,'imshow',side_effect=lambda title,frame:shown.append(frame.copy())),patch.object(cv,'resizeWindow'),patch.object(cv,'waitKey',return_value=0),patch.object(cv,'getWindowProperty',return_value=1):camera.alignment_camera_worker(None,None,{'updates':updates,'title':'test'})
  np.testing.assert_array_equal(raw,saved);self.assertIs(stream.render_preview,original);self.assertEqual(shown[0].shape,(440,1280,3));self.assertTrue(np.any(np.all(shown[0][42:402,:640]==[0,255,255],axis=2)))
if __name__=='__main__':
 result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(AlignmentTests));r=dict(status='passed' if result.wasSuccessful() else 'failed',tests_run=result.testsRun,failures=len(result.failures),errors=len(result.errors),devices_opened=False,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),controller_hashes=plan.hashes());(plan.OUT/'software_admission_result.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');sys.exit(not result.wasSuccessful())
