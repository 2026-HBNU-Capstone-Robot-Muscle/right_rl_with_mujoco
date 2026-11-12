"""One actual placement, independent photographs of both held endpoints."""
import numpy as np
from leap_reference_speed_yaw_20261029 import ReferenceCapture
from yaw_roundtrip_plan_20261030 import LOADED
class YawRoundtripCapture(ReferenceCapture):
    def place(self,bus,command,report,guard):
        print('큐브는 한 번만 놓습니다. 정회전 뒤 계속 잡은 채 역회전하며, REMOVE까지 손을 대지 마세요.',flush=True)
        super().place(bus,command,report,guard)
        report.update(continuous_two_leg_trial=True,requires_no_cube_replacement_between_legs=True,only_one_place_checkpoint_programmed=True,physical_roundtrip_verified=False,
            camera_instruction='PLACE: GREEN up, ORANGE wristward; then FORWARD + INVERSE without touching')
    def rotation_frame(self,bus,command,report,state):
        z=np.load(LOADED)
        if np.array_equal(command,z['forward_hold_goal']):leg='forward';photo='cube_forward_turn_hold.jpg';message='정회전 고정 구간입니다. 큐브를 계속 잡고 역회전합니다. 손을 대지 마세요.'
        elif np.array_equal(command,z['inverse_hold_goal']):leg='inverse';photo='cube_inverse_turn_hold.jpg';message='역회전 고정 구간입니다. 자동 OPEN 뒤 REMOVE에서만 큐브를 빼주세요.'
        else:raise RuntimeError('Unreviewed roundtrip hold endpoint')
        starts=getattr(self,'roundtrip_hold_starts',{})
        if leg not in starts:
            starts[leg]=self.now();self.roundtrip_hold_starts=starts
            report.update(roundtrip_leg=leg,camera_instruction=('FORWARD HOLD; INVERSE follows; HANDS CLEAR' if leg=='forward' else 'INVERSE HOLD; automatic OPEN follows; HANDS CLEAR'),physical_inverse90_verified=False)
            report[leg+'_hold_goal_ticks']=np.asarray(command).tolist();report[leg+'_hold_position_ticks']=state['position'].tolist();print(message,flush=True)
            if leg=='inverse':report.update(rotation_goal_ticks=np.asarray(command).tolist(),rotation_position_ticks=state['position'].tolist(),physical_horizontal_90_verified=False)
        key=leg+'_observation_photo'
        if not report.get(key):
            saved=self.camera.capture_named(photo,after=starts[leg]+.2)
            if saved is not None:report[key]=saved
