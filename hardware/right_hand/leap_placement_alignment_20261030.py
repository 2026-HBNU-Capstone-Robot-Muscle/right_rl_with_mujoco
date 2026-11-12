"""User-launched alignment capture. Default opens no devices."""
import sys
import numpy as np
from placement_alignment_plan_20261030 import PlacementAlignmentPlan,REQUEST,PROOF
from placement_alignment_gate_20261030 import validate_review,run_screen
from placement_alignment_camera_20261030 import AlignmentCamera
from placement_alignment_capture_20261030 import capture
from leap_four_reply_yaw_20261029 import FourReplyBus
from leap_reference_speed_yaw_20261029 import runtime,confirmed_run
class AlignmentBus(FourReplyBus):
 def approve(self,plan,proof):
  self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start;self.screen_approved=True;self.support_checked=False;self.root_release_threshold=np.max(np.asarray([v['root_clear_goal_ticks'] for v in self.proof['empty_recovery']['static_cases']]),axis=0)+4;self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
 def write(self,addr,size,values,verify=True):
  if addr==116 and self.report.get('placement_goals_locked'):raise RuntimeError('Position writes are blocked during placement-only capture')
  return super().write(addr,size,values,verify=verify)
PlacementAlignmentPlan.CAMERA_TYPE=AlignmentCamera
def main():
 try:validate_review(PlacementAlignmentPlan(REQUEST),PROOF)
 except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:print('NOT READY:',exc,'. No devices opened.');return 2
 print('첫 SPACE 빈손 준비 -> PLACE에서 배치 -> 두 번째 SPACE 사진 -> 큐브 제거 -> 세 번째 SPACE OFF.',flush=True)
 print('PLACE에서 왼쪽 화면의 노란 큐브 윤곽에 모서리를 맞춰보세요. 잡기와 회전은 실행하지 않습니다.',flush=True)
 import prepared_placement_capture as protocol
 old_run,old_capture=runtime.run,protocol.capture
 try:runtime.run=confirmed_run;protocol.capture=capture;return runtime.main(PlacementAlignmentPlan,bus_type=AlignmentBus,screener=run_screen)
 finally:runtime.run=old_run;protocol.capture=old_capture
if __name__=='__main__':raise SystemExit(main())
