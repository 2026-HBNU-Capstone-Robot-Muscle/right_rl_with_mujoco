"""Natural hands-free placement; same paced health reads and three SPACEs."""
import numpy as np
from leap_reference_speed_yaw_20261029 import ReferenceCapture
from new_pair_grasp_capture_20261027 import NewPairGraspCapture
class NaturalCradleCapture(ReferenceCapture):
 def place(self,bus,command,report,guard):
  print('큐브는 손을 떼도 남아 있던 손가락 뿌리 홈에 자연스럽게 놓으세요. 기울어져 있어도 됩니다.',flush=True)
  print('초록 위, 주황 손목 쪽입니다. 이전 노란 윤곽의 높이나 위치에 맞추지 않습니다.',flush=True)
  print('이번 동작은 새 접근으로 잡고5초 유지한 뒤 자동으로 엽니다. 회전 지령은 없습니다.',flush=True)
  NewPairGraspCapture.place(self,bus,command,report,guard)
  report.update(placement_basis='user-confirmed hands-free tilted root recess',elevated_reference_matching_required=False,physical_grasp_verified=False,physical_rotation_verified=False)
 def hold_frame(self,bus,command,report,state):
  if not getattr(self,'hold_started',False):
   self.hold_started=True;self.hold_began=self.now();report.update(camera_instruction='GRASP HOLD5s; hands clear; automatic OPEN follows. NO TURN.',grasp_position_ticks=state['position'].tolist(),grasp_goal_ticks=np.asarray(command).tolist());print('새 파지5초 관찰. 손을 대지 마세요. 자동 OPEN 뒤 큐브를 제거합니다.',flush=True)
  if not report.get('grasp_observation_photo'):
   photo=self.camera.capture_named('cube_grasp_hold.jpg',after=self.hold_began+.2)
   if photo is not None:report['grasp_observation_photo']=photo
