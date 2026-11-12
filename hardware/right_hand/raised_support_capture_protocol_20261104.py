import numpy as np
from leap_reference_speed_yaw_20261029 import ReferenceCapture
from raised_cradle_plan_20261104 import validate_candidate
class RaisedSupportCapture(ReferenceCapture):
 def require_open(self,command):
  if not np.array_equal(command,validate_candidate()['start']):raise RuntimeError('Raised support pose not reached')
 def place(self,bus,command,report,guard):
  self.require_open(command);self.guard=guard;bus.grasp_protocol=self;observe=self._observer(bus,command,report,guard);state,photo=self._photo('empty_support.jpg','empty_support',observe);report.update(empty_support_photo=photo,camera_instruction='PLACE 초록 위/주황 손목. 자연스럽게 놓고 손을 뗀 뒤 두번째 SPACE. 촬영만 진행.')
  print('PLACE: 초록 위, 주황 손목 쪽. 새로 준비한 받침에서 큐브가 스스로 놓이는 곳에 자연스럽게 놓으세요.',flush=True)
  print('높은 위치에 손으로 맞추지 마세요. 손을 떼면 떨어지거나 놓을 수 없으면 Q.',flush=True)
  print('손을 뗀 뒤 두번째 SPACE: 사진과2초 정지 관찰만 저장합니다. 이후 손가락 각도는 바뀌지 않습니다.',flush=True);self._space('place_cube',observe);state,photo=self._photo('cube_on_support.jpg','place_cube',observe);report.update(placement_photo=photo,cube_placement_confirmed=True,cube_present=True,placement_position_ticks=state['position'].tolist(),placement_goal_ticks=np.asarray(command).tolist(),registration_verified=False,post_placement_motion=False)
 def finish(self,bus,command,report):
  self.require_open(command)
  if not report.get('cube_placement_confirmed') or not hasattr(bus,'pending') or bus.pending:raise RuntimeError('Static observation queue incomplete')
  observe=self._observer(bus,command,report,self.guard);state,photo=self._photo('cube_after_static_hold.jpg','support_observation',observe);report.update(support_observation_photo=photo,camera_instruction='촬영 완료. 큐브를 빼고 세번째 SPACE = OFF.',physical_grasp_verified=False,physical_rotation_verified=False)
  print('촬영 완료. 큐브만 빼고 세번째 SPACE를 누르면 토크 OFF합니다.',flush=True);state=self._space('remove_cube',observe);report.update(final_on_state={k:v.tolist() for k,v in state.items()},cube_removal_confirmed=True,cube_removal_confirmation_source='operator_SPACE',cube_present=False)
