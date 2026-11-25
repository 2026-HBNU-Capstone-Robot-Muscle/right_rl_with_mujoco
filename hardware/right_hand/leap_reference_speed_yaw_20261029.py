"""Operator-started reproduction of the completed GREEN-up75ms yaw."""
import numpy as np
from reference_speed_yaw_plan_20261029 import ReferenceSpeedYawPlan,REQUEST,PROOF
from reference_speed_yaw_gate_20261029 import validate_review,run_screen
from leap_paced_retained_yaw_20261029 import PacedCapture,PacedBus,PairGraspCamera,runtime,confirmed_run
from new_pair_grasp_capture_20261027 import NewPairGraspCapture
class ReferenceCapture(PacedCapture):
    def place(self,bus,command,report,guard):
        print('이번 기준 방향: 초록 위, 주황 손목ID41 쪽. 이전 완료 실행과 같은 방향입니다.',flush=True)
        print('배치 사진: calibrated_quarter_trial_20261028_174723_390729\\cube_before_grasp.jpg',flush=True)
        NewPairGraspCapture.place(self,bus,command,report,guard)
        report.update(placement_orientation='GREEN up; ORANGE wristward',placement_reference='calibrated_quarter_trial_20261028_174723_390729')
class ReferenceBus(PacedBus):
    def approve(self,plan,proof):
        self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start;self.screen_approved=True;self.support_checked=False
        self.root_release_threshold=np.max(np.asarray([v['root_clear_goal_ticks'] for v in self.proof['empty_recovery']['static_cases']]),axis=0)+4
        self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
ReferenceSpeedYawPlan.CAPTURE_PROTOCOL_TYPE=ReferenceCapture;ReferenceSpeedYawPlan.CAMERA_TYPE=PairGraspCamera
def main():
    try:validate_review(ReferenceSpeedYawPlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:print('NOT READY:',exc,'. No devices opened.');return 2
    print('이전 완료 실행의 속도 재현: 빈손 첫 SPACE -> PLACE에서 초록 위/주황 손목ID41 쪽 -> 두 번째 SPACE.',flush=True)
    print('이전75ms 회전 지령을 그대로 사용합니다. 관절 힘과 중단 기준을 높이지 않습니다.',flush=True)
    print('회전 뒤 자동 OPEN. REMOVE에서 큐브를 뺀 뒤 세 번째 SPACE. Q/Esc/Ctrl+C로 중단.',flush=True)
    print('배치 대기75ms, OFF 설정 전체 고속 읽기와200ms 정상 상태2회 확인은 유지합니다. 실제 회전량은 영상으로 확인합니다.',flush=True)
    saved=runtime.run
    try:runtime.run=confirmed_run;return runtime.main(ReferenceSpeedYawPlan,bus_type=ReferenceBus,screener=run_screen)
    finally:runtime.run=saved
if __name__=='__main__':raise SystemExit(main())
