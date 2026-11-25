"""Default is device-free; --execute plus terminal SPACE starts EMPTY motion."""
import numpy as np
from final_grip_bus_20261116 import StandardGripBus as ForwardHoldBus
from leap_reference_speed_yaw_20261029 import runtime, confirmed_run
from leap_empty_hold_comm_20261029 import EmptyCamera
from final_grip_pose_plan_20261116 import FinalGripPosePlan, REQUEST, PROOF
from final_grip_pose_gate_20261116 import validate_review, run_screen
from final_grip_pose_capture_20261116 import FinalGripPoseCapture

class FinalGripPoseBus(ForwardHoldBus):
    def approve(self, plan, proof):
        self.proof=validate_review(plan,proof)
        self.plan_module=plan; self.planner=plan.validate_start
        self.screen_approved=True; self.support_checked=False
        self.root_release_threshold=np.max(np.asarray([
            r['root_clear_goal_ticks'] for r in self.proof['empty_recovery']['static_cases']]),axis=0)+4
        self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']

FinalGripPosePlan.CAMERA_TYPE=EmptyCamera
FinalGripPosePlan.CAPTURE_PROTOCOL_TYPE=FinalGripPoseCapture

def run_main():
    try:
        validate_review(FinalGripPosePlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:
        print('NOT READY:',exc,'. No devices opened.',flush=True)
        return 2
    print('빈손 전용입니다. 처음 SPACE -> 현재 자세 검사 -> 천천히 최종 손 모양 -> 계속 유지.',flush=True)
    print('회전·큐브 배치·자동 OPEN 없음. 유지 중 SPACE 종료/OFF. Q/Esc/Ctrl+C 중단.',flush=True)
    print('450mA/기존PID/최대2틱75ms 및 기존 건강·통신·추종 기준과 전체500초 한도를 유지합니다.',flush=True)
    previous=runtime.run
    try:
        runtime.run=confirmed_run
        return runtime.main(FinalGripPosePlan,bus_type=FinalGripPoseBus,screener=run_screen)
    finally:
        runtime.run=previous

def main():
    import sys
    from contextlib import nullcontext
    from final_grip_single_instance_20261116 import SingleInstance
    from final_grip_pose_plan_20261116 import ROOT
    lock=SingleInstance(ROOT/'.final_grip_pose.lock') if '--execute' in sys.argv else nullcontext()
    try:
        with lock:return run_main()
    except RuntimeError as exc:
        print('NOT STARTED:',exc,flush=True);return 2

if __name__=='__main__':
    raise SystemExit(main())
