"""Operator-started supported cube observation; default opens no devices."""
import numpy as np
from supported_inverse_trial_plan_20261029 import SupportedInverseTrialPlan,REQUEST,PROOF
from supported_inverse_trial_gate_20261029 import validate_review,run_screen
from supported_inverse_trial_capture_20261029 import SupportedInverseCapture
from leap_four_reply_yaw_20261029 import FourReplyBus,PairGraspCamera,runtime,confirmed_run
class SupportedInverseBus(FourReplyBus):
    def approve(self,plan,proof):
        self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start
        self.screen_approved=True;self.support_checked=False
        self.root_release_threshold=np.max(np.asarray([r['root_clear_goal_ticks'] for r in self.proof['empty_recovery']['static_cases']]),axis=0)+4
        self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
        self.report.update(experimental_supported_inverse_observation=True,physical_inverse_90_verified=False,
            placement_orientation='GREEN up; WHITE wristward',nominal_model_inverse_deg=self.proof['supported_inverse_cube_models']['nominal_placed_to_hold_deg'])
SupportedInverseTrialPlan.CAPTURE_PROTOCOL_TYPE=SupportedInverseCapture;SupportedInverseTrialPlan.CAMERA_TYPE=PairGraspCamera
def main():
    try:validate_review(SupportedInverseTrialPlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:
        print('NOT READY:',exc,'. No devices opened.');return 2
    print('빈손 완주한 새 경로로 큐브 역회전을 관찰합니다. 실제 회전각은 저장 영상에서 확인합니다.',flush=True)
    print('첫 SPACE는 반드시 빈손. 큐브는 PLACE가 나올 때까지 손과 카메라 밖에 둡니다.',flush=True)
    print('PLACE에서 초록 위/흰 면 손목 ID41 쪽. 손을 뺀 뒤 두 번째 SPACE.',flush=True)
    print('자동 OPEN 뒤 REMOVE에서 큐브를 뺀 다음 세 번째 SPACE로 OFF합니다.',flush=True)
    print('450mA/PID/75ms와 전압·전류·온도·추종·진행 중단은 그대로입니다. Q/Esc/Ctrl+C 중단.',flush=True)
    saved=runtime.run
    try:runtime.run=confirmed_run;return runtime.main(SupportedInverseTrialPlan,bus_type=SupportedInverseBus,screener=run_screen)
    finally:runtime.run=saved
if __name__=='__main__':raise SystemExit(main())
