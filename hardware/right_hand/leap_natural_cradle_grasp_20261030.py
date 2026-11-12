"""Only a user --execute invocation may move the hand. Default opens nothing."""
import numpy as np
from natural_cradle_grasp_plan_20261030 import NaturalCradleGraspPlan,REQUEST,PROOF
from natural_cradle_grasp_gate_20261030 import validate_review,run_screen
from natural_cradle_grasp_capture_20261030 import NaturalCradleCapture
from leap_four_reply_yaw_20261029 import FourReplyBus
from leap_reference_speed_yaw_20261029 import runtime,confirmed_run
from leap_new_pair_grasp_20261027 import PairGraspCamera
class NaturalCamera(PairGraspCamera):
 PRESERVE_EXPOSURE=True
class NaturalBus(FourReplyBus):
 def approve(self,plan,proof):
  self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start;self.screen_approved=True;self.support_checked=False;self.root_release_threshold=np.max(np.asarray([v['root_clear_goal_ticks'] for v in self.proof['empty_recovery']['static_cases']]),axis=0)+4;self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
NaturalCradleGraspPlan.CAMERA_TYPE=NaturalCamera;NaturalCradleGraspPlan.CAPTURE_PROTOCOL_TYPE=NaturalCradleCapture
def main():
 try:validate_review(NaturalCradleGraspPlan(REQUEST),PROOF)
 except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:print('NOT READY:',exc,'. No devices opened.');return 2
 print('첫SPACE 빈손 -> PLACE에서 자연스러운 홈에 큐브/손빼고두번째SPACE -> 새파지/5초고정/자동OPEN -> 큐브빼고세번째SPACE OFF.',flush=True)
 print('초록위/주황손목. 기울어져 있어도 됩니다. 노란 윤곽 맞추기는 없습니다. 이번 동작은 파지 확인이며 회전은 없습니다.',flush=True)
 saved=runtime.run
 try:runtime.run=confirmed_run;return runtime.main(NaturalCradleGraspPlan,bus_type=NaturalBus,screener=run_screen)
 finally:runtime.run=saved
if __name__=='__main__':raise SystemExit(main())
