"""One operator SPACE; same joint movement EMPTY only, automatic OFF."""
import numpy as np
from yaw_load_empty_plan_20261029 import YawLoadEmptyPlan,REQUEST,PROOF
from yaw_load_empty_gate_20261029 import validate_review,run_screen
from leap_four_reply_yaw_20261029 import FourReplyBus,runtime,confirmed_run
from leap_empty_hold_comm_20261029 import EmptyCamera
from new_pair_grasp_capture_20261027 import NewPairGraspCapture
class EmptyPrefixProtocol(NewPairGraspCapture):
    def release(self,*args):raise RuntimeError('Unexpected EMPTY release callback')
    def finish(self,bus,command,report):
        report.update(cube_present=False,empty_reference_prefix_complete=True)
        print('빈손 비교 구간 완료. 큐브를 놓지 않습니다. 자동 OFF와 설정 복구를 진행합니다.',flush=True)
class EmptyPrefixBus(FourReplyBus):
    def approve(self,plan,proof):
        self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start;self.screen_approved=True;self.support_checked=False
        self.root_release_threshold=np.max(np.asarray([v['root_clear_goal_ticks'] for v in self.proof['empty_recovery']['static_cases']]),axis=0)+4
        self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
        self.report.update(cube_present=False,diagnostic_condition='EMPTY throughout; no placement')
YawLoadEmptyPlan.CAPTURE_PROTOCOL_TYPE=EmptyPrefixProtocol;YawLoadEmptyPlan.CAMERA_TYPE=EmptyCamera
def main():
    try:validate_review(YawLoadEmptyPlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:print('NOT READY:',exc,'. No devices opened.');return 2
    print('끝까지 빈손입니다. 큐브를 손과 카메라 시야 밖에 두고 터미널 SPACE 한 번만 누릅니다.',flush=True)
    print('같은 잡기와 회전 관절 구간을 빈손으로 비교한 뒤 자동 OFF합니다. PLACE나 추가 SPACE는 없습니다.',flush=True)
    print('기존4개씩 전체 상태 읽기, 전류/PID/전압/추종/온도와 중단 기준을 유지합니다. Q/Esc/Ctrl+C로 중단.',flush=True)
    saved=runtime.run
    try:runtime.run=confirmed_run;return runtime.main(YawLoadEmptyPlan,bus_type=EmptyPrefixBus,screener=run_screen)
    finally:runtime.run=saved
if __name__=='__main__':raise SystemExit(main())
