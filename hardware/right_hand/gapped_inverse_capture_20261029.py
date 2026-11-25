"""Three genuine operator checkpoints; new placement endpoint and face reference."""
import numpy as np
from leap_paced_retained_yaw_20261029 import PacedCapture
from gapped_inverse_plan_20261029 import placement_goal,OPEN_PATH

class GappedInverseCapture(PacedCapture):
    def require_open(self,command):
        expected=np.load(OPEN_PATH)['open_goal'] if getattr(self,'placement_done',False) else placement_goal()
        if not np.array_equal(command,expected):
            raise RuntimeError('Reviewed placement/removal endpoint not reached; no checkpoint prompt')
    def place(self,bus,command,report,guard):
        self.require_open(command);self.guard=guard;bus.grasp_protocol=self
        observe=self._observer(bus,command,report,guard)
        state,photo=self._photo('open_before_cube.jpg','open_before_cube',observe)
        report.update(open_before_cube_photo=photo,open_before_cube_state={k:v.tolist() for k,v in state.items()},
            camera_instruction='PLACE: GREEN up, WHITE toward WRIST ID41; hands clear then SECOND SPACE',
            placement_orientation='GREEN up; WHITE wristward',physical_inverse_cube_rotation_verified=False)
        print('PLACE CUBE: 초록 위, 흰 면은 손목 ID41 쪽. 엄지와 중지 사이 손바닥 위 빈 공간에 놓으세요.',flush=True)
        print('배치 그림: right_hand_rotation_20261029\\gapped_inverse_cube\\placement_views.png',flush=True)
        print('큐브가 자연스럽게 받쳐져야 합니다. 맞지 않으면 밀어 넣지 말고 Q. ID40은 가려져도 됩니다.',flush=True)
        print('손을 뺀 뒤 터미널의 두 번째 SPACE: 틈을 천천히 닫고 역회전 경로를 실행합니다.',flush=True)
        self._space('place_cube',observe)
        state,photo=self._photo('cube_before_grasp.jpg','place_cube',observe)
        report.update(placement_photo=photo,placement_position_ticks=state['position'].tolist(),
            placement_goal_ticks=np.asarray(command).tolist(),cube_placement_confirmed=True,
            cube_present=True,registration_verified=False)
        self.placement_done=True
        for n in (3,2,1):
            print(f'역회전 시험 시작 {n}초 전. 손을 빼주세요. Q로 중단.',flush=True)
            began=self.now()
            while self.now()-began<1:observe('loaded_countdown');self.sleep(.02)
    def hold_frame(self,bus,command,report,state):
        if not getattr(self,'hold_started',False):
            self.hold_started=True;self.hold_began=self.now()
            report.update(camera_instruction='GRASP HOLD; reverse638 follows; hands clear',
                grasp_position_ticks=state['position'].tolist(),grasp_goal_ticks=np.asarray(command).tolist())
            print('닫기 완료. 잠시 고정한 뒤 역방향 관절 경로를 실행합니다. 손을 대지 마세요.',flush=True)
        if not report.get('grasp_observation_photo'):
            photo=self.camera.capture_named('cube_before_horizontal_turn.jpg',after=self.hold_began+.2)
            if photo is not None:report['grasp_observation_photo']=photo
    def rotation_frame(self,bus,command,report,state):
        if not getattr(self,'rotation_hold_started',False):
            self.rotation_hold_started=True;self.rotation_hold_began=self.now()
            report.update(camera_instruction='INVERSE HOLD; automatic OPEN follows; hands clear',
                rotation_goal_ticks=np.asarray(command).tolist(),rotation_position_ticks=state['position'].tolist(),
                physical_horizontal_90_verified=False,physical_inverse_cube_rotation_verified=False)
            print('역회전 지령 완료. 고정 후 자동으로 엽니다. 실물 회전량은 저장 영상으로 확인합니다.',flush=True)
        if not report.get('rotation_observation_photo'):
            photo=self.camera.capture_named('cube_horizontal_turn_hold.jpg',after=self.rotation_hold_began+.2)
            if photo is not None:report['rotation_observation_photo']=photo
