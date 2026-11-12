"""Only the user's explicit --execute call can access motion hardware."""
import numpy as np
from forward_hold_plan_20261104 import ForwardHoldPlan,REQUEST,PROOF
from forward_hold_gate_20261104 import validate_review,run_screen
from forward_hold_capture_20261104 import ForwardHoldCapture
from leap_four_reply_yaw_20261029 import FourReplyBus
from leap_reference_speed_yaw_20261029 import runtime,confirmed_run
from leap_natural_cradle_grasp_20261030 import NaturalCamera
from verified_off_restore_context_20261105 import post_off_restore
class ForwardHoldBus(FourReplyBus):
    def write(self,addr,size,values,verify=True):
        with post_off_restore(self,addr,values):
            return super().write(addr,size,values,verify=verify)
    def approve(self,plan,proof):
        self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start;self.screen_approved=True;self.support_checked=False
        self.root_release_threshold=np.max(np.asarray([v['root_clear_goal_ticks'] for v in self.proof['empty_recovery']['static_cases']]),axis=0)+4;self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
ForwardHoldPlan.CAMERA_TYPE=NaturalCamera;ForwardHoldPlan.CAPTURE_PROTOCOL_TYPE=ForwardHoldCapture
def main():
    try:validate_review(ForwardHoldPlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:print('NOT READY:',exc,'. No devices opened.');return 2
    print('빈손 첫SPACE -> PLACE 초록 위/주황 손목 -> 손빼고 두번째SPACE -> 기존 정회전/10초 고정.',flush=True)
    print('자동OPEN/역회전 없음. 마지막에 큐브 아래를 받치고 세번째SPACE=OFF. 종료 뒤 큐브를 빼세요.',flush=True)
    print('450mA/PID/75ms와 기존 전압·통신·추종 중단 기준 유지. Q/Esc/Ctrl+C 중단.',flush=True)
    saved=runtime.run
    try:runtime.run=confirmed_run;return runtime.main(ForwardHoldPlan,bus_type=ForwardHoldBus,screener=run_screen)
    finally:runtime.run=saved
if __name__=='__main__':raise SystemExit(main())
