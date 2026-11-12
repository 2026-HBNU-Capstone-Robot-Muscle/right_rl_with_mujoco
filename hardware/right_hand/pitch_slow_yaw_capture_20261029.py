"""One placement, uninterrupted pitch/transfer/yaw, then removal; fresh SPACE."""
import numpy as np
from new_pair_grasp_capture_20261027 import NewPairGraspCapture
from pitch_handoff_plan_20261028 import pitch_start
from new_pair_grasp_plan_20261027 import OPEN_PATH

class PitchSlowYawCapture(NewPairGraspCapture):
    def place(self,bus,command,report,guard):
        if not np.array_equal(command,pitch_start()):raise RuntimeError('Pitch start endpoint missing; no PLACE')
        self.guard=guard;bus.grasp_protocol=self
        observe=self._observer(bus,command,report,guard)
        state,photo=self._photo('pitch_setup_empty.jpg','pitch_setup_empty',observe)
        report.update(prepared_empty_photo=photo,prepared_empty_ticks=state['position'].tolist(),
            camera_instruction='PLACE only now: GREEN up / ORANGE toward wrist ID41')
        print('\n[큐브 놓기] 지금 손은 정지해 기다리는 중입니다.',flush=True)
        print('큐브를 이전 성공 사진 위치에 놓으세요. 초록이 위, 주황이 손목 ID41 쪽입니다.',flush=True)
        print('큐브가 스스로 놓인 것을 보고 손을 완전히 뺀 뒤, 터미널에서 두 번째 SPACE를 누르세요.',flush=True)
        print('SPACE를 누른 뒤에는 뒤집기와 수평 회전이 끝날 때까지 큐브에 손대지 마세요. 맞지 않으면 Q로 종료합니다.',flush=True)
        self._space('place_cube',observe)
        state,photo=self._photo('cube_before_pitch.jpg','place_cube',observe)
        report.update(placement_photo=photo,placement_position_ticks=state['position'].tolist(),
            placement_goal_ticks=np.asarray(command).tolist(),cube_placement_confirmed=True,cube_present=True,registration_verified=False)
        for n in (3,2,1):
            print(f'{n}초 뒤 뒤집기 시작. 손을 떼고 기다려 주세요. Q로 중단.',flush=True)
            began=self.now()
            while self.now()-began<1:observe('loaded_countdown');self.sleep(.02)
    def pitch_frame(self,bus,command,report,state):
        if not hasattr(self,'pitch_began'):
            self.pitch_began=self.now()
            report.update(pitch_goal_ticks=np.asarray(command).tolist(),pitch_position_ticks=state['position'].tolist())
            print('뒤집기 목표 완료. 잠시 유지한 뒤 다음 손 자세로 이동합니다.',flush=True)
        if not report.get('pitch_observation_photo'):
            photo=self.camera.capture_named('cube_pitch_hold.jpg',after=self.pitch_began+.2)
            if photo is not None:report['pitch_observation_photo']=photo
    def transfer_frame(self,bus,command,report,state):
        if not hasattr(self,'transfer_began'):
            self.transfer_began=self.now()
            report.update(transfer_goal_ticks=np.asarray(command).tolist(),transfer_position_ticks=state['position'].tolist())
            print('자세 이동 목표 완료. 큐브를 그대로 둡니다. 다음은 느린 수평 회전입니다.',flush=True)
        if not report.get('transfer_observation_photo'):
            photo=self.camera.capture_named('cube_transfer_hold.jpg',after=self.transfer_began+.2)
            if photo is not None:report['transfer_observation_photo']=photo
    def hold_frame(self,bus,command,report,state):
        if not hasattr(self,'grasp_began'):
            self.grasp_began=self.now()
            report.update(grasp_position_ticks=state['position'].tolist(),grasp_goal_ticks=np.asarray(command).tolist())
            print('다시 잡는 동작 완료. 잠시 유지한 뒤 절반 속도로 수평 회전합니다.',flush=True)
        if not report.get('grasp_observation_photo'):
            photo=self.camera.capture_named('cube_before_horizontal_turn.jpg',after=self.grasp_began+.2)
            if photo is not None:report['grasp_observation_photo']=photo
    def rotation_frame(self,bus,command,report,state):
        if not hasattr(self,'yaw_began'):
            self.yaw_began=self.now()
            report.update(rotation_goal_ticks=np.asarray(command).tolist(),rotation_position_ticks=state['position'].tolist(),physical_horizontal_90_verified=False)
            print('수평 회전 목표 완료. 잠시 유지한 뒤 자동으로 손을 폅니다. REMOVE가 뜰 때까지 기다려 주세요.',flush=True)
        if not report.get('rotation_observation_photo'):
            photo=self.camera.capture_named('cube_horizontal_turn_hold.jpg',after=self.yaw_began+.2)
            if photo is not None:report['rotation_observation_photo']=photo
    def finish(self,bus,command,report):
        if not report.get('cube_placement_confirmed'):raise RuntimeError('No explicit placement')
        if not np.array_equal(command,np.load(OPEN_PATH)['open_goal']):raise RuntimeError('Final opening incomplete')
        if not hasattr(bus,'pending') or bus.pending:raise RuntimeError('Queue incomplete')
        observe=self._observer(bus,command,report,self.guard)
        state,photo=self._photo('cube_after_open.jpg','opened_before_remove',observe)
        report.update(opened_before_remove_photo=photo,physical_handoff_verified=False,physical_flip_verified=False,
            physical_horizontal_90_verified=False,camera_instruction='REMOVE cube, then THIRD SPACE for OFF')
        print('\n[REMOVE CUBE] 이제 큐브를 빼세요. 뺀 뒤에만 터미널에서 세 번째 SPACE를 누르세요.',flush=True)
        state=self._space('remove_cube',observe)
        report.update(final_on_state={k:v.tolist() for k,v in state.items()},cube_removal_confirmed=True,cube_present=False)
