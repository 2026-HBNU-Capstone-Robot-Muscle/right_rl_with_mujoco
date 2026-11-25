"""Support-before-OFF checkpoint. Never asks for joint changes while powered."""
import numpy as np
from leap_reference_speed_yaw_20261029 import ReferenceCapture
from new_pair_grasp_capture_20261027 import NewPairGraspCapture
from manual_grasp_plan_20261106 import validate_candidate
class ManualGraspCapture(ReferenceCapture):
    def require_open(self,command):
        if not np.array_equal(command,validate_candidate()['start']):raise RuntimeError('Reviewed placement pose not reached')
    def place(self,bus,command,report,guard):
        print('PLACE 표시에서 초록 위/주황 손목 쪽으로 자연스럽게 놓습니다.',flush=True)
        print('같은 파지까지만 진행합니다. 자동 열기 없음. 토크OFF 확인 전에는 손가락 각도를 바꾸지 마세요.',flush=True)
        NewPairGraspCapture.place(self,bus,command,report,guard)
        report.update(automatic_open=False,automatic_inverse=False,manual_joint_teaching=True,manual_joint_correction_allowed=False)
    def hold_frame(self,bus,command,report,state):
        if not getattr(self,'hold_started',False):
            self.hold_started=True;self.hold_began=self.now();print('이전 파지 자세입니다.5초 관찰 후 같은 목표로 대기합니다. 자동으로 열지 않습니다.',flush=True)
            report.update(grasp_position_ticks=state['position'].tolist(),grasp_goal_ticks=np.asarray(command).tolist(),camera_instruction='GRASP HOLD. NO AUTO OPEN. Wait for SUPPORT + SPACE OFF.')
        if not report.get('grasp_observation_photo'):
            photo=self.camera.capture_named('cube_grasp_hold.jpg',after=self.hold_began+.2)
            if photo is not None:report['grasp_observation_photo']=photo
    def finish(self,bus,command,report):
        z=validate_candidate()
        if not report.get('cube_placement_confirmed') or not np.array_equal(command,z['goals'][-1]) or not hasattr(bus,'pending') or bus.pending:raise RuntimeError('Grasp/hold queue incomplete; no manual teaching prompt')
        observe=self._observer(bus,command,report,self.guard);state,photo=self._photo('grasp_before_torque_off.jpg','grasp_before_torque_off',observe)
        report.update(before_off_photo=photo,before_off_state={k:v.tolist() for k,v in state.items()},camera_instruction='SUPPORT CUBE from below. SPACE = torque OFF. Do not reshape joints yet.')
        print('\n[파지 완료 / 자동 열기 없음]',flush=True)
        print('큐브를 아래에서 받친 뒤 세 번째 SPACE를 누르세요. 그 다음 토크OFF와 설정 복구를 확인합니다.',flush=True)
        print('아직 손가락을 밀어 바꾸지 마세요. 다음 화면의 [토크OFF 보정] 표시 뒤에 각도를 바꿉니다.',flush=True)
        state=self._space('support_cube_for_off',observe)
        report.update(final_on_state={k:v.tolist() for k,v in state.items()},cube_support_confirmed_before_off=True,cube_present=True,cube_removal_confirmed=False,manual_joint_correction_allowed=False)
