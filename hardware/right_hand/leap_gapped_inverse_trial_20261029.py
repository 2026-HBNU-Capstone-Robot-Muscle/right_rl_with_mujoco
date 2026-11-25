"""User-started loaded inverse observation. Default opens no devices."""
import numpy as np
from gapped_inverse_plan_20261029 import GappedInversePlan,REQUEST,PROOF
from gapped_inverse_gate_20261029 import validate_review,run_screen
from gapped_inverse_capture_20261029 import GappedInverseCapture
from leap_four_reply_yaw_20261029 import FourReplyBus,PairGraspCamera,runtime,confirmed_run

class GappedInverseBus(FourReplyBus):
    def approve(self,plan,proof):
        self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start
        self.screen_approved=True;self.support_checked=False
        self.root_release_threshold=np.max(np.asarray([r['root_clear_goal_ticks'] for r in self.proof['empty_recovery']['static_cases']]),axis=0)+4
        self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
        self.report.update(experimental_inverse_observation=True,physical_inverse_cube_rotation_verified=False,
            inverse_90_ready=False,placement_orientation='GREEN up; WHITE wristward',
            nominal_model_inverse_deg=self.proof['inverse_model']['nominal_placed_to_turn_deg'])

GappedInversePlan.CAPTURE_PROTOCOL_TYPE=GappedInverseCapture
GappedInversePlan.CAMERA_TYPE=PairGraspCamera

def main():
    try:validate_review(GappedInversePlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:
        print('NOT READY:',exc,'. No devices opened.');return 2
    print('역회전 관찰 시험입니다. 모델 약27~39도이며 90도 완성 시험은 아닙니다.',flush=True)
    print('첫 SPACE는 빈손. 큐브는 PLACE가 나올 때까지 손과 카메라 밖에 둡니다.',flush=True)
    print('PLACE에서 초록 위/흰 면 손목 ID41 쪽. 손을 빼고 두 번째 SPACE.',flush=True)
    print('자동 OPEN 뒤 REMOVE에서 큐브를 뺀 다음 세 번째 SPACE로 OFF합니다.',flush=True)
    print('기존450mA/PID/75ms와 모든 건강·추적·진행 중단을 유지합니다. Q/Esc/Ctrl+C로 중단.',flush=True)
    saved=runtime.run
    try:runtime.run=confirmed_run;return runtime.main(GappedInversePlan,bus_type=GappedInverseBus,screener=run_screen)
    finally:runtime.run=saved

if __name__=='__main__':raise SystemExit(main())
