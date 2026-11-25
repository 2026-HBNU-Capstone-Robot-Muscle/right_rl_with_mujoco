"""Raw camera unchanged; side-by-side guidance is rendered in the camera process."""
from pathlib import Path
import queue
REFERENCE=Path(__file__).resolve().parent/'single_forward_trial_20261030_052636_068402/cube_before_grasp.jpg'
def comparison_frame(cv,frame,reference,instruction):
 import numpy as np
 display=np.zeros((440,1280,3),dtype=np.uint8)
 display[42:402,:640]=cv.resize(frame,(640,360));display[42:402,640:]=cv.resize(reference,(640,360))
 cv.putText(display,'LIVE - CURRENT HAND',(15,28),cv.FONT_HERSHEY_SIMPLEX,.65,(0,255,255),2)
 cv.putText(display,'REFERENCE - BEFORE OBSERVED TURN',(652,28),cv.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),2)
 cv.putText(display,instruction[:145],(10,426),cv.FONT_HERSHEY_SIMPLEX,.48,(0,255,255),1)
 return display
def alignment_camera_worker(send,stop,preview=None):
 import cv2 as cv,numpy as np
 import camera_stream as stream
 reference=cv.imdecode(np.frombuffer(REFERENCE.read_bytes(),np.uint8),cv.IMREAD_COLOR)
 if reference is None:send.send(('error','Saved placement reference could not be decoded'));send.close();return
 original=stream.render_preview
 def render(cv,frame,preview,instruction):
  while True:
   try:instruction=preview['updates'].get_nowait()
   except queue.Empty:break
  display=comparison_frame(cv,frame,reference,instruction)
  if not preview.get('alignment_window_size_set'):
   cv.resizeWindow(preview['title'],1200,460);preview['alignment_window_size_set']=True
  cv.imshow(preview['title'],display);key=cv.waitKey(1)&255;closed=cv.getWindowProperty(preview['title'],cv.WND_PROP_VISIBLE)<1
  return instruction,key in (ord('q'),ord('Q'),27) or closed
 try:
  stream.render_preview=render;stream.camera_worker(send,stop,preview)
 finally:stream.render_preview=original

from leap_new_pair_grasp_20261027 import PairGraspCamera
class AlignmentCamera(PairGraspCamera):
 PRESERVE_EXPOSURE=True
 def __init__(self,folder,report):
  from empty_trial_camera import EmptyTrialCamera
  from empty_start_marker_check import make_empty_detector,EmptyStartEvidence
  self.empty_detector=make_empty_detector();self.empty_evidence=EmptyStartEvidence();self.empty_checked_stamp=None;self.empty_streak=0;self.empty_quality=None;self.restoration_path=Path(folder).parent/'camera_exposure_worker.json'
  report.update(camera_requested_exposure=None,camera_exposure_control='unchanged',marker_visibility_is_motion_gate=False,empty_start_marker_screen_is_empty_hand_proof=False,placement_comparison_reference=str(REFERENCE),comparison_display_is_3D_registration=False)
  EmptyTrialCamera.__init__(self,folder,report,worker=alignment_camera_worker)
  print('비교 화면 왼쪽은 현재, 오른쪽은 이전 회전 실행의 배치 사진입니다. 첫 SPACE는 빈손으로 누르세요.',flush=True)
