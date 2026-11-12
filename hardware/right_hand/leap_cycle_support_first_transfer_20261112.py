"""User-started support-first transfer trial; default mode opens no devices."""
import numpy as np
from leap_forward_hold_observe_20261104 import ForwardHoldBus
from leap_reference_speed_yaw_20261029 import runtime,confirmed_run
from leap_natural_cradle_grasp_20261030 import NaturalCamera
from support_first_trial_plan_20261112 import SupportFirstTrialPlan,REQUEST,PROOF,ACTIVE,HOLDS,scope_approved
from support_first_trial_gate_20261112 import validate_review,run_screen
from support_first_trial_capture_20261112 import SupportFirstTrialCapture
BASE_SAMPLE=ForwardHoldBus.checked_sample

class SupportFirstTrialBus(ForwardHoldBus):
    def approve(self,plan,proof):
        if not scope_approved():raise RuntimeError('Support-first scope mismatch; no writes')
        self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start
        self.screen_approved=True;self.support_checked=False
        self.root_release_threshold=np.max(np.asarray([r['root_clear_goal_ticks'] for r in self.proof['empty_recovery']['static_cases']]),axis=0)+4
        self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
    @staticmethod
    def checked_sample(bus,command,report,phase,**kwargs):
        state,errors=BASE_SAMPLE(bus,command,report,phase,**kwargs)
        if phase in (*ACTIVE,*HOLDS) and hasattr(bus,'grasp_protocol'):bus.grasp_protocol.transfer_frame(bus,command,report,phase)
        return state,errors

SupportFirstTrialPlan.CAMERA_TYPE=NaturalCamera;SupportFirstTrialPlan.CAPTURE_PROTOCOL_TYPE=SupportFirstTrialCapture

def main():
    try:validate_review(SupportFirstTrialPlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:
        print('NOT READY:',exc,'. No devices opened.',flush=True);return 2
    print(SupportFirstTrialPlan.TITLE,flush=True)
    print('첫SPACE 빈손 검사/준비 -> PLACE 초록위·주황손목 -> 두 번째SPACE -> 새 전환/유지 -> 받치고 마지막SPACE OFF.',flush=True)
    print('450mA/기존PID/2틱75ms와 기존 건강·추종·통신 기준, 승인된 최대500초 유지.500 재입력 없음.',flush=True)
    print('실제 낙하 방지·파지와 윗단 공간은 이번 저장 영상으로 확인합니다. Q/Esc/Ctrl+C 중단.',flush=True)
    previous=runtime.run
    try:runtime.run=confirmed_run;return runtime.main(SupportFirstTrialPlan,bus_type=SupportFirstTrialBus,screener=run_screen)
    finally:runtime.run=previous

if __name__=='__main__':raise SystemExit(main())
