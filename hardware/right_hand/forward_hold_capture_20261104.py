"""Three operator checkpoints; terminal OFF before cube removal."""
import numpy as np
from leap_reference_speed_yaw_20261029 import ReferenceCapture
from new_pair_grasp_capture_20261027 import NewPairGraspCapture
from forward_hold_plan_20261104 import validate_candidate
class ForwardHoldCapture(ReferenceCapture):
    def require_open(self,command):
        if not np.array_equal(command,validate_candidate()['start']):raise RuntimeError('Observed source placement pose not reached')
    def place(self,bus,command,report,guard):
        print('PLACE에서만 큐브를 놓습니다. 초록 위/주황 손목ID41 방향. 손을 떼도 놓여 있어야 합니다.',flush=True)
        print('이번에는 기존 정회전 뒤10초 동안 같은 목표각을 유지하고 멈춥니다. 자동OPEN이나 역회전은 없습니다.',flush=True)
        NewPairGraspCapture.place(self,bus,command,report,guard)
        report.update(placement_orientation='GREEN up; ORANGE wristward',physical90_verified=False,physical_body_hold_verified=False,automatic_open=False,automatic_inverse=False)
    def hold_frame(self,bus,command,report,state):
        if not getattr(self,'hold_started',False):self.hold_started=True;self.hold_began=self.now()
        if not report.get('grasp_observation_photo'):
            photo=self.camera.capture_named('cube_grasp_hold.jpg',after=self.hold_began+.2)
            if photo is not None:report['grasp_observation_photo']=photo
    def finish(self,bus,command,report):
        z=validate_candidate()
        if not report.get('cube_placement_confirmed') or not np.array_equal(command,z['goals'][-1]) or not hasattr(bus,'pending') or bus.pending:raise RuntimeError('Forward/fixed-hold queue incomplete')
        observe=self._observer(bus,command,report,self.guard);state,photo=self._photo('cube_after_fixed_hold.jpg','fixed_hold_observation',observe)
        report.update(fixed_hold_observation_photo=photo,fixed_hold_seconds=10.05,physical90_verified=False,physical_body_hold_verified=False,camera_instruction='10초 관찰 완료. 큐브를 아래에서 받치고 세번째 SPACE=OFF. 종료 후 큐브 제거.')
        print('정회전과10초 고정 관찰이 끝났습니다. 큐브를 아래에서 받친 뒤 세번째 SPACE로 토크 OFF합니다.',flush=True)
        print('프로그램 종료/OFF 확인 뒤 큐브를 빼세요. 손가락을 억지로 벌려 꺼내지 마세요. Q도 중단합니다.',flush=True)
        state=self._space('support_cube_for_off',observe);report.update(final_on_state={k:v.tolist() for k,v in state.items()},cube_support_confirmed_before_off=True,cube_removal_confirmed=False,cube_present=True)
