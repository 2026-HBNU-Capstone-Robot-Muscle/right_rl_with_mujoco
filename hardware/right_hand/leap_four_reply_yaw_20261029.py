"""Operator-started original trajectory with smaller strict health replies."""
import numpy as np
from four_reply_yaw_plan_20261029 import FourReplyYawPlan,REQUEST,PROOF
from four_reply_yaw_gate_20261029 import validate_review,run_screen,WINDOW_POLICY
from leap_reference_speed_yaw_20261029 import ReferenceBus,ReferenceCapture,PairGraspCamera,runtime,confirmed_run
from fast_health_read4_20261029 import block_once as fast4,POLICY
class FourReplyBus(ReferenceBus):
    def _block_once(self,address,length,fields):
        if (address,length)!=(64,83):return super()._block_once(address,length,fields)
        self.report['active_health_read_policy']=dict(POLICY)
        try:return fast4(self,address,length,fields)
        finally:self.report['sync_receive_window_policy']=dict(WINDOW_POLICY)
    def preflight(self):
        state=super().preflight();self.report['serial_link_policy'].update(full_health_reply_group_size=4)
        self.report['active_health_read_policy']=dict(POLICY);return state
    def approve(self,plan,proof):
        self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start;self.screen_approved=True;self.support_checked=False
        self.root_release_threshold=np.max(np.asarray([v['root_clear_goal_ticks'] for v in self.proof['empty_recovery']['static_cases']]),axis=0)+4
        self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
FourReplyYawPlan.CAPTURE_PROTOCOL_TYPE=ReferenceCapture;FourReplyYawPlan.CAMERA_TYPE=PairGraspCamera
def main():
    try:validate_review(FourReplyYawPlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:print('NOT READY:',exc,'. No devices opened.');return 2
    print('빈손 첫 SPACE -> PLACE에서 초록 위/주황 손목ID41 쪽 -> 손을 빼고 두 번째 SPACE.',flush=True)
    print('같은 이전 완료 회전 경로입니다. 회전 후 자동 OPEN. REMOVE에서 큐브를 빼고 세 번째 SPACE.',flush=True)
    print('건강 상태는4개씩356바이트 응답으로 나눠 읽되,16개 전체 확인과 기존200ms 정상 상태2회 확인은 유지합니다.',flush=True)
    print('기존 전류/PID/전압/온도/추종 및 반복 수신 실패 중단 기준은 유지합니다. Q/Esc/Ctrl+C로 중단.',flush=True)
    print('모터OFF 비교는 통과했으며 구동 중 개선 여부와 실제 회전량은 아직 미확인입니다.',flush=True)
    saved=runtime.run
    try:runtime.run=confirmed_run;return runtime.main(FourReplyYawPlan,bus_type=FourReplyBus,screener=run_screen)
    finally:runtime.run=saved
if __name__=='__main__':raise SystemExit(main())
