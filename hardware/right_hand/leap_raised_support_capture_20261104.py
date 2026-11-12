import numpy as np
from raised_support_capture_plan_20261104 import RaisedSupportCapturePlan,REQUEST,PROOF
from raised_support_capture_gate_20261104 import validate_review,run_screen
from raised_support_capture_protocol_20261104 import RaisedSupportCapture
from leap_four_reply_yaw_20261029 import FourReplyBus
from leap_reference_speed_yaw_20261029 import runtime,confirmed_run
from leap_natural_cradle_grasp_20261030 import NaturalCamera
class CaptureBus(FourReplyBus):
 def approve(self,plan,proof):
  self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start;self.screen_approved=True;self.support_checked=False;self.root_release_threshold=np.max(np.asarray([v['root_clear_goal_ticks'] for v in self.proof['empty_recovery']['static_cases']]),axis=0)+4;self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
RaisedSupportCapturePlan.CAMERA_TYPE=NaturalCamera;RaisedSupportCapturePlan.CAPTURE_PROTOCOL_TYPE=RaisedSupportCapture
def main():
 try:validate_review(RaisedSupportCapturePlan(REQUEST),PROOF)
 except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:print('NOT READY:',exc,'. No devices opened.');return 2
 print('새 받침 확인만 합니다. 빈손 첫SPACE -> 준비 -> PLACE에서 큐브/손빼고 두번째SPACE ->2초 촬영 -> 큐브빼고 세번째SPACE OFF.',flush=True)
 print('큐브를 놓은 뒤 추가 각도 변경과 회전은 없습니다. 웹캠은 자동으로 열리고 로그와 사진은 자동 저장됩니다.',flush=True)
 saved=runtime.run
 try:runtime.run=confirmed_run;return runtime.main(RaisedSupportCapturePlan,bus_type=CaptureBus,screener=run_screen)
 finally:runtime.run=saved
if __name__=='__main__':raise SystemExit(main())
