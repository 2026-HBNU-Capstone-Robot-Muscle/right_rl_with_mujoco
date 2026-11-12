"""Only the user --execute invocation can actuate; default opens no device."""
import numpy as np
from raised_cradle_plan_20261104 import RaisedCradlePlan,REQUEST,PROOF
from raised_cradle_gate_20261104 import validate_review,run_screen
from raised_cradle_capture_20261104 import RaisedCradleCapture
from leap_four_reply_yaw_20261029 import FourReplyBus
from leap_reference_speed_yaw_20261029 import runtime,confirmed_run
from leap_natural_cradle_grasp_20261030 import NaturalCamera
class RaisedCradleBus(FourReplyBus):
 def approve(self,plan,proof):
  self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start;self.screen_approved=True;self.support_checked=False;self.root_release_threshold=np.max(np.asarray([v['root_clear_goal_ticks'] for v in self.proof['empty_recovery']['static_cases']]),axis=0)+4;self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
RaisedCradlePlan.CAMERA_TYPE=NaturalCamera;RaisedCradlePlan.CAPTURE_PROTOCOL_TYPE=RaisedCradleCapture
def main():
 try:validate_review(RaisedCradlePlan(REQUEST),PROOF)
 except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:print('NOT READY:',exc,'. No devices opened.');return 2
 print('빈손 첫SPACE -> 자동 받침 준비 -> PLACE에서 초록 위/주황 손목, 손을 뗀 뒤 두번째SPACE -> 전체 정회전/5초 관찰 -> 받치고 세번째SPACE OFF -> 큐브 제거.',flush=True)
 print('모델은 약65도입니다. 실물90도 성공은 아직 확인하지 않았습니다. 힘/전압/추적/통신 중단 기준은 그대로입니다.',flush=True)
 saved=runtime.run
 try:runtime.run=confirmed_run;return runtime.main(RaisedCradlePlan,bus_type=RaisedCradleBus,screener=run_screen)
 finally:runtime.run=saved
if __name__=='__main__':raise SystemExit(main())
