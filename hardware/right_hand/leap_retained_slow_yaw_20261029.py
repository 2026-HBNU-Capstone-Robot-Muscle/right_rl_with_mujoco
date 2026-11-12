"""User-started isolated slow yaw with the scoped quiet-time calculation fix."""
import numpy as np
from retained_slow_yaw_plan_20261029 import RetainedSlowYawPlan,REQUEST,PROOF
from retained_slow_yaw_gate_20261029 import validate_review,run_screen
from leap_calibrated_quarter_20261028 import CalibratedQuarterBus,HorizontalQuarterCapture,PairGraspCamera,runtime,confirmed_run
from combined_quiet_reserve_20261029 import receive,POLICY
from fast_health_read_20261029 import block_once as fast_block,POLICY as FAST_POLICY
from fast_health_window_20261029 import WINDOW_POLICY
from calibrated_precise_clock_20261028 import install as install_clock
from horizontal_serial_link_20261028 import prepare_host
class RetainedCapture(HorizontalQuarterCapture):
    def place(self,bus,command,report,guard):
        self.require_open(command);self.guard=guard;bus.grasp_protocol=self
        observe=self._observer(bus,command,report,guard)
        state,photo=self._photo('open_before_cube.jpg','open_before_cube',observe)
        report.update(open_before_cube_photo=photo,open_before_cube_state={k:v.tolist() for k,v in state.items()},
            camera_instruction='PLACE RED up; GREEN toward wrist ID41; this trial does not pitch')
        print('\n[PLACE CUBE] 이번에는 빨강이 위입니다.',flush=True)
        print('빨강 중앙 ID2가 위, 초록 중앙 ID5가 손목 ID41 쪽입니다. 손바닥 ID40 옆의 엄지 쪽 공간에 예시 사진처럼 놓으세요.',flush=True)
        print('위치 사진: pitch_slow_yaw_20261029_004532_520638\\cube_transfer_hold.jpg',flush=True)
        print('비교 그림: right_hand_rotation_20261029\\retained_slow_yaw_v1\\placement_comparison.svg',flush=True)
        print('큐브가 스스로 놓이면 손을 빼고 두 번째 SPACE. 들어가지 않거나 손으로 받쳐야 하면 Q.',flush=True)
        self._space('place_cube',observe)
        state,photo=self._photo('cube_before_grasp.jpg','place_cube',observe)
        report.update(placement_photo=photo,placement_position_ticks=state['position'].tolist(),placement_goal_ticks=np.asarray(command).tolist(),
            cube_placement_confirmed=True,cube_present=True,registration_verified=False,placement_orientation='RED up; GREEN wristward')
        for n in (3,2,1):
            print(f'{n}초 뒤 잡기와 수평 회전 시작. 손을 떼고 기다려 주세요. Q로 중단.',flush=True);began=self.now()
            while self.now()-began<1:observe('loaded_countdown');self.sleep(.02)
class RetainedBus(CalibratedQuarterBus):
    def preflight(self):
        prepare_host(self);install_clock(self);firmware=[]
        for motor in range(1,17):
            version=self.read(motor,6)
            if version<46:raise RuntimeError(f'ID{motor} firmware{version} does not support Fast READ; no settings changed')
            firmware.append(dict(id=motor,firmware=version))
        self.report['fast_read_live_firmware']=firmware
        state=super().preflight()
        self.report['serial_link_policy'].update(full_health_reply_group_size=8,health_read_instruction=0x8a)
        return state
    def _block_once(self,address,length,fields):
        try:
            if (address,length)==(64,83):return fast_block(self,address,length,fields)
            return super()._block_once(address,length,fields)
        finally:self.report['sync_receive_window_policy']=dict(WINDOW_POLICY)
    def _receive(self,operation,label,request):
        if request.get('kind')=='sync_read' and request.get('address')==64 and request.get('length')==83:
            self.report['read_resynchronization_policy']=dict(POLICY)
            return receive(self,operation,label,request)
        return super()._receive(operation,label,request)
    def approve(self,plan,proof):
        self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start
        self.screen_approved=True;self.support_checked=False
        self.root_release_threshold=np.max(np.asarray([v['root_clear_goal_ticks'] for v in self.proof['empty_recovery']['static_cases']]),axis=0)+4
        self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
RetainedSlowYawPlan.CAPTURE_PROTOCOL_TYPE=RetainedCapture
RetainedSlowYawPlan.CAMERA_TYPE=PairGraspCamera
def main():
    try:validate_review(RetainedSlowYawPlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:
        print('NOT READY:',exc,'. No devices opened.',flush=True);return 2
    print('이번에는 수평 구간만 확인합니다. 첫 SPACE는 빈손. PLACE 뒤 빨강 위/초록 손목 ID41 쪽으로 놓고 두 번째 SPACE.',flush=True)
    print('뒤집기는 실행하지 않습니다. 약4~5분. 자동으로 손을 편 뒤 REMOVE에서 큐브를 빼고 세 번째 SPACE.',flush=True)
    print('통신 수신 정리와 정상 전체 상태2회 확인을 같은200ms 안에 마칩니다.450mA/PID/전압/추종 기준은 유지합니다.',flush=True)
    print('공식 Fast Sync Read로8개씩 묶어 전체16개 건강 상태를 읽습니다. 통신속도와 모터 설정은 바꾸지 않습니다.',flush=True)
    print('실험용이며 실제 회전량과 파지 유지는 저장 영상으로 확인합니다. Q/Esc/Ctrl+C로 중단.',flush=True)
    previous=runtime.run
    try:runtime.run=confirmed_run;return runtime.main(RetainedSlowYawPlan,bus_type=RetainedBus,screener=run_screen)
    finally:runtime.run=previous
if __name__=='__main__':raise SystemExit(main())
