"""Only an explicit operator --execute and terminal SPACE can move the hand."""
import numpy as np
from middle_probe_plan_20261106 import MiddleProbePlan,REQUEST,PROOF,HOLD
from middle_probe_gate_20261106 import validate_review,run_screen
from middle_probe_capture_20261106 import MiddleProbeCapture
from leap_forward_hold_observe_20261104 import ForwardHoldBus
from leap_reference_speed_yaw_20261029 import runtime,confirmed_run
from leap_natural_cradle_grasp_20261030 import NaturalCamera
BASE_SAMPLE=ForwardHoldBus.checked_sample
class MiddleProbeBus(ForwardHoldBus):
    def approve(self,plan,proof):
        self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start;self.screen_approved=True;self.support_checked=False;self.root_release_threshold=np.max(np.asarray([v['root_clear_goal_ticks'] for v in self.proof['empty_recovery']['static_cases']]),axis=0)+4;self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
    @staticmethod
    def checked_sample(bus,command,report,phase,**kwargs):
        state,errors=BASE_SAMPLE(bus,command,report,phase,**kwargs)
        if phase==HOLD and hasattr(bus,'grasp_protocol'):bus.grasp_protocol.middle_frame(bus,command,report,state)
        return state,errors
MiddleProbePlan.CAMERA_TYPE=NaturalCamera;MiddleProbePlan.CAPTURE_PROTOCOL_TYPE=MiddleProbeCapture
def main():
    try:validate_review(MiddleProbePlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:print('NOT READY:',exc,'. No devices opened.',flush=True);return 2
    print('첫 SPACE는 빈손. PLACE 뒤 초록 위/주황 손목, 손을 떼고 두 번째 SPACE.',flush=True)
    print('기존 파지에서 엄지는 고정하고 중지만 접었다 돌아옵니다. 직접 손가락을 만지지 마세요.',flush=True)
    print('자동 OPEN 뒤 큐브를 빼고 세 번째 SPACE OFF. 이번에는 회전과 수동 각도 보정이 없습니다.',flush=True)
    saved=runtime.run
    try:runtime.run=confirmed_run;return runtime.main(MiddleProbePlan,bus_type=MiddleProbeBus,screener=run_screen)
    finally:runtime.run=saved
if __name__=='__main__':raise SystemExit(main())
