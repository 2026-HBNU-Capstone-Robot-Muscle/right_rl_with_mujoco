"""Operator-only loaded yaw; inherited placement/removal and unchanged settings."""
import numpy as np
from adaptive_yaw_loaded_plan_20261029 import AdaptiveYawLoadedPlan,REQUEST,PROOF
from adaptive_yaw_loaded_gate_20261029 import validate_review,run_screen
from leap_four_reply_yaw_20261029 import FourReplyBus,ReferenceCapture,PairGraspCamera,runtime
from root_lag_pause_20261029 import RootLagPause,POLICY
from root_lag_motion_runner_20261029 import run
class AdaptiveLoadedBus(FourReplyBus):
    def approve(self,plan,proof):
        self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start;self.screen_approved=True;self.support_checked=False
        self.root_release_threshold=np.max(np.asarray([v['root_clear_goal_ticks'] for v in self.proof['empty_recovery']['static_cases']]),axis=0)+4
        self.initial_pair_checked=not self.proof['initial_self_separation']['enabled'];self.root_lag_pause=RootLagPause()
        self.report.update(root_lag_pause_policy=dict(POLICY),actual_wait_effect_not_yet_verified=True)
AdaptiveYawLoadedPlan.CAPTURE_PROTOCOL_TYPE=ReferenceCapture;AdaptiveYawLoadedPlan.CAMERA_TYPE=PairGraspCamera
def main():
    try:validate_review(AdaptiveYawLoadedPlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:print('NOT READY:',exc,'. No devices opened.');return 2
    print('첫 SPACE는 빈손. PLACE가 뜨면 초록 위, 주황은 손목 ID41 쪽으로 놓습니다.',flush=True)
    print('손을 뺀 뒤 두 번째 SPACE: 잡기와 이전638개 회전 목표를 실행합니다. 실제 회전량은 저장 영상으로 확인합니다.',flush=True)
    print('중지가 뒤처지면 현재 목표에서 잠깐 기다립니다. 대기당1.2초/총20초, 기존300초에 포함합니다.',flush=True)
    print('전류/PID/전압/온도/진행/0.18rad 추종 중단은 그대로입니다. 따라오지 못하면 OFF합니다.',flush=True)
    print('회전 후 자동 OPEN. REMOVE에서 큐브를 뺀 뒤 세 번째 SPACE로 OFF합니다. Q/Esc/Ctrl+C 중단.',flush=True)
    saved=runtime.run
    try:runtime.run=run;return runtime.main(AdaptiveYawLoadedPlan,bus_type=AdaptiveLoadedBus,screener=run_screen)
    finally:runtime.run=saved
if __name__=='__main__':raise SystemExit(main())
