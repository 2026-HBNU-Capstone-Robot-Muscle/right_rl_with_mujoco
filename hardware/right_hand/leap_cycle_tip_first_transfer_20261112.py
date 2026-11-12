"""User-started support-first transfer trial; default mode opens no devices."""
import numpy as np
from leap_forward_hold_observe_20261104 import ForwardHoldBus
from leap_reference_speed_yaw_20261029 import runtime,confirmed_run
from leap_natural_cradle_grasp_20261030 import NaturalCamera
from tip_first_trial_plan_20261112 import TipFirstTrialPlan,REQUEST,PROOF,ACTIVE,HOLDS,scope_approved
from tip_first_trial_gate_20261112 import validate_review,run_screen
from tip_first_trial_capture_20261112 import TipFirstTrialCapture
BASE_SAMPLE=ForwardHoldBus.checked_sample

class TipFirstTrialBus(ForwardHoldBus):
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

TipFirstTrialPlan.CAMERA_TYPE=NaturalCamera;TipFirstTrialPlan.CAPTURE_PROTOCOL_TYPE=TipFirstTrialCapture

def main():
    try:validate_review(TipFirstTrialPlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:
        print('NOT READY:',exc,'. No devices opened.',flush=True);return 2
    print(TipFirstTrialPlan.TITLE,flush=True)
    print('첫SPACE 빈손 검사/준비 -> PLACE 초록위·주황손목 -> 두 번째SPACE -> 새 전환/유지 -> 받치고 마지막SPACE OFF.',flush=True)
    print('450mA/기존PID/2틱75ms와 기존 건강·추종·통신 기준, 승인된 최대500초 유지.500 재입력 없음.',flush=True)
    print('이번 파일은 끝마디부터 굽혀 큐브를 옮기는 시험입니다. 이전에 멈춘 약지 닫힘과 다음 회전·왼손 작업은 포함하지 않습니다.',flush=True)
    print('큐브가 미끄러지거나 떨어지면 Q/Esc/Ctrl+C로 중단하세요. 모터 기준만으로 낙하를 자동 판정하지 않습니다.',flush=True)
    previous=runtime.run
    try:runtime.run=confirmed_run;return runtime.main(TipFirstTrialPlan,bus_type=TipFirstTrialBus,screener=run_screen)
    finally:runtime.run=previous

if __name__=='__main__':raise SystemExit(main())
