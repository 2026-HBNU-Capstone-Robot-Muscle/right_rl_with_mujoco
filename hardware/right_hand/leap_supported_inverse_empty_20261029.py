"""Operator-only first EMPTY execution of the new supported inverse route."""
import numpy as np
from supported_inverse_empty_plan_20261029 import SupportedInverseEmptyPlan,REQUEST,PROOF
from supported_inverse_empty_gate_20261029 import validate_review,run_screen
from leap_yaw_load_empty_20261029 import EmptyPrefixProtocol,EmptyCamera,runtime,FourReplyBus,confirmed_run
class SupportedEmptyBus(FourReplyBus):
    def approve(self,plan,proof):
        self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start
        self.screen_approved=True;self.support_checked=False
        self.root_release_threshold=np.max(np.asarray([r['root_clear_goal_ticks'] for r in self.proof['empty_recovery']['static_cases']]),axis=0)+4
        self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
        self.report.update(cube_present=False,diagnostic_condition='EMPTY supported inverse trajectory; cube never placed',physical_inverse_90_verified=False)
SupportedInverseEmptyPlan.CAPTURE_PROTOCOL_TYPE=EmptyPrefixProtocol;SupportedInverseEmptyPlan.CAMERA_TYPE=EmptyCamera
def main():
    try:validate_review(SupportedInverseEmptyPlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:
        print('NOT READY:',exc,'. No devices opened.');return 2
    print('이번에는 끝까지 빈손입니다. 큐브를 손과 카메라 밖에 두세요. PLACE는 없습니다.',flush=True)
    print('검지 받침과 더 큰 엄지·중지 회전 경로를 처음 실물 빈손으로 확인합니다.',flush=True)
    print('손 모양은 그대로 두고 터미널 SPACE 한 번. 준비, 새 경로, OPEN 뒤 자동 OFF합니다.',flush=True)
    print('기존450mA/PID/75ms와 전압·전류·온도·추종·진행 중단을 유지합니다. Q/Esc/Ctrl+C 중단.',flush=True)
    saved=runtime.run
    try:runtime.run=confirmed_run;return runtime.main(SupportedInverseEmptyPlan,bus_type=SupportedEmptyBus,screener=run_screen)
    finally:runtime.run=saved
if __name__=='__main__':raise SystemExit(main())
