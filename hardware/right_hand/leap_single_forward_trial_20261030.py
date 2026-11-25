"""Operator-only single forward; original bus and health policies."""
import numpy as np
from single_forward_plan_20261030 import SingleForwardPlan,REQUEST,PROOF
from single_forward_gate_20261030 import validate_review,run_screen
from single_forward_capture_20261030 import SingleForwardCapture
from leap_four_reply_yaw_20261029 import FourReplyBus
from leap_reference_speed_yaw_20261029 import PairGraspCamera,runtime,confirmed_run
class SingleForwardBus(FourReplyBus):
 def approve(self,plan,proof):
  self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start;self.screen_approved=True;self.support_checked=False
  self.root_release_threshold=np.max(np.asarray([v['root_clear_goal_ticks'] for v in self.proof['empty_recovery']['static_cases']]),axis=0)+4
  self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
SingleForwardPlan.CAPTURE_PROTOCOL_TYPE=SingleForwardCapture;SingleForwardPlan.CAMERA_TYPE=PairGraspCamera
def main():
 try:validate_review(SingleForwardPlan(REQUEST),PROOF)
 except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:print('NOT READY:',exc,'. No devices opened.');return 2
 print('정회전 한 동작 재현: 빈손 첫SPACE -> PLACE에서 초록 위/주황 손목ID41 쪽 -> 두 번째SPACE.',flush=True)
 print('정회전 뒤 자동OPEN -> 큐브를 빼고 세 번째SPACE로OFF. 실제 회전량은 저장 영상에서 확인합니다.',flush=True)
 print('기존450mA/PID/75ms와 전압/온도/추종/통신 중단 기준을 사용합니다. Q/Esc/Ctrl+C로 중단.',flush=True)
 saved=runtime.run
 try:runtime.run=confirmed_run;return runtime.main(SingleForwardPlan,bus_type=SingleForwardBus,screener=run_screen)
 finally:runtime.run=saved
if __name__=='__main__':raise SystemExit(main())
