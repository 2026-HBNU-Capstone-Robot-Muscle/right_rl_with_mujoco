"""Photos of source grip and middle-folded grip while the other fingers hold."""
import numpy as np
from leap_reference_speed_yaw_20261029 import ReferenceCapture
from new_pair_grasp_capture_20261027 import NewPairGraspCapture
class MiddleProbeCapture(ReferenceCapture):
    def place(self,bus,command,report,guard):
        print('PLACE: 손가락 뿌리 홈에 자연스럽게 놓으세요. 초록 위, 주황 손목 ID41 쪽. 억지로 넣지 마세요.',flush=True)
        print('손을 뗀 뒤 두 번째 SPACE: 기존 파지 -> 엄지는 고정하고 중지만 접기/5초 -> 되돌린 뒤 자동 OPEN.',flush=True)
        print('손가락을 직접 만져 보정하지 마세요. 큐브 회전 명령은 없습니다.',flush=True)
        NewPairGraspCapture.place(self,bus,command,report,guard)
        report.update(placement_basis='user-confirmed hands-free natural root recess',physical_rotation_verified=False,new_common_grip_verified=False,manual_joint_teaching=False)
    def hold_frame(self,bus,command,report,state):
        if not getattr(self,'hold_started',False):
            self.hold_started=True;self.hold_began=self.now();report.update(camera_instruction='기존 파지 고정. 다음에는 중지만 접습니다. 손을 떼고 기다리세요.',grasp_position_ticks=state['position'].tolist(),grasp_goal_ticks=np.asarray(command).tolist());print('기존 파지 5초 관찰. 다음은 엄지 고정/중지만 접기입니다.',flush=True)
        if not report.get('grasp_observation_photo'):
            photo=self.camera.capture_named('cube_grasp_hold.jpg',after=self.hold_began+.2)
            if photo is not None:report['grasp_observation_photo']=photo
    def middle_frame(self,bus,command,report,state):
        if not hasattr(self,'middle_began'):
            self.middle_began=self.now();report.update(camera_instruction='중지 보정 자세 5초. 큐브와 손가락에 손대지 마세요. 되돌린 뒤 자동 OPEN.',middle_probe_goal_ticks=np.asarray(command).tolist());print('중지 보정 자세 5초 관찰. 엄지 고정. 손을 대지 마세요. 다음은 복귀/OPEN입니다.',flush=True)
        if not report.get('middle_probe_photo'):
            photo=self.camera.capture_named('middle_probe_hold.jpg',after=self.middle_began+1.)
            if photo is not None:report.update(middle_probe_photo=photo,middle_probe_position_ticks=state['position'].tolist(),middle_probe_snapshot={k:v.tolist() for k,v in state.items()})
