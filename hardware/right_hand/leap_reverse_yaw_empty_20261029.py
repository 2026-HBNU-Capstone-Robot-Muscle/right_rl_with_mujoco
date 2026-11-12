"""Operator-started EMPTY reverse joint check. No cube placement or live policy."""
import numpy as np
from reverse_yaw_empty_plan_20261029 import ReverseYawEmptyPlan,REQUEST,PROOF
from reverse_yaw_empty_gate_20261029 import validate_review,run_screen
from leap_yaw_load_empty_20261029 import EmptyPrefixProtocol,EmptyCamera,runtime,FourReplyBus,confirmed_run
class ReverseEmptyBus(FourReplyBus):
    def approve(self,plan,proof):
        self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start;self.screen_approved=True;self.support_checked=False
        self.root_release_threshold=np.max(np.asarray([v['root_clear_goal_ticks'] for v in self.proof['empty_recovery']['static_cases']]),axis=0)+4
        self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
        self.report.update(cube_present=False,diagnostic_condition='EMPTY reverse joint cycle; no cube placement',original_forward_targets_played_in_reverse=True)
ReverseYawEmptyPlan.CAPTURE_PROTOCOL_TYPE=EmptyPrefixProtocol;ReverseYawEmptyPlan.CAMERA_TYPE=EmptyCamera
def main():
    try:validate_review(ReverseYawEmptyPlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:print('NOT READY:',exc,'. No devices opened.');return 2
    print('끝까지 빈손입니다. 큐브는 손과 카메라 시야 밖에 둡니다. PLACE는 없습니다.',flush=True)
    print('이번에는 반대방향 경로 확인입니다. 기존에 도달한 관절 목표를 역순으로 움직입니다.',flush=True)
    print('터미널 SPACE 한 번으로 준비/역방향 경로/고정 후 자동 OFF합니다. Q/Esc/Ctrl+C 중단.',flush=True)
    print('기존 전류/PID/전압/온도/추종/진행 검사와75ms 간격을 유지합니다. 큐브 역회전은 아직 검증하지 않았습니다.',flush=True)
    saved=runtime.run
    try:runtime.run=confirmed_run;return runtime.main(ReverseYawEmptyPlan,bus_type=ReverseEmptyBus,screener=run_screen)
    finally:runtime.run=saved
if __name__=='__main__':raise SystemExit(main())
