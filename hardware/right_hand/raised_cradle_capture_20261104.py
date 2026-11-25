"""Natural placement, reference turn, supported OFF; no automatic untwisting."""
import numpy as np
from leap_reference_speed_yaw_20261029 import ReferenceCapture
from new_pair_grasp_capture_20261027 import NewPairGraspCapture
from raised_cradle_plan_20261104 import validate_candidate
class RaisedCradleCapture(ReferenceCapture):
 def require_open(self,command):
  if not np.array_equal(command,validate_candidate()['start']):raise RuntimeError('Raised support endpoint not reached')
 def place(self,bus,command,report,guard):
  print('받침 준비가 끝났습니다. 초록 위/주황 손목 쪽으로, 큐브가 스스로 남는 홈에 자연스럽게 놓으세요.',flush=True)
  print('손으로 높은 위치에 맞추지 마세요. 손을 떼도 놓여 있을 때 두번째 SPACE. 자연스럽게 놓이지 않으면 Q.',flush=True)
  NewPairGraspCapture.place(self,bus,command,report,guard);report.update(placement_basis='naturally seated on prepared raised support; no yellow alignment',rotation_reference_only=True,model_predicted_angle_deg=65.4,physical90_verified=False)
 def hold_frame(self,bus,command,report,state):
  if not getattr(self,'hold_started',False):self.hold_started=True;self.hold_began=self.now();report['camera_instruction']='파지 관찰 후 전체 정회전; 손을 떼고 기다리세요.'
  if not report.get('grasp_observation_photo'):
   photo=self.camera.capture_named('cube_grasp_hold.jpg',after=self.hold_began+.2)
   if photo is not None:report['grasp_observation_photo']=photo
 def finish(self,bus,command,report):
  if not report.get('cube_placement_confirmed') or not np.array_equal(command,validate_candidate()['goals'][-1]) or not hasattr(bus,'pending') or bus.pending:raise RuntimeError('Turn queue incomplete; OFF')
  observe=self._observer(bus,command,report,self.guard);state,photo=self._photo('cube_after_turn.jpg','turn_observation',observe);report.update(turn_observation_photo=photo,physical90_verified=False,camera_instruction='회전 종료. 큐브가 떨어지지 않게 아래에서 받치고 세번째 SPACE = 토크 OFF. 꺼진 뒤 큐브 제거.')
  print('회전 시퀀스와5초 관찰이 끝났습니다. 자동으로 손을 펴거나 반대로 돌리지 않습니다.',flush=True)
  print('큐브가 떨어지지 않게 아래에서 받친 다음 세번째 SPACE로 토크 OFF. 종료 후 큐브를 빼세요. Q로도 중단.',flush=True);state=self._space('support_cube_for_off',observe);report.update(final_on_state={k:v.tolist() for k,v in state.items()},cube_support_confirmed_before_off=True,cube_removal_confirmed=False,cube_present=True)
