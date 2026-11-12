"""Operator-only continuous Cube yaw experiment. Default opens no devices."""
import numpy as np
from yaw_roundtrip_plan_20261030 import YawRoundtripPlan,REQUEST,PROOF
from yaw_roundtrip_gate_20261030 import validate_review,run_screen
from yaw_roundtrip_capture_20261030 import YawRoundtripCapture
from leap_four_reply_yaw_20261029 import FourReplyBus,PairGraspCamera,runtime,confirmed_run
class YawRoundtripBus(FourReplyBus):
    def approve(self,plan,proof):
        self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start;self.screen_approved=True;self.support_checked=False
        self.root_release_threshold=np.max(np.asarray([r['root_clear_goal_ticks'] for r in self.proof['empty_recovery']['static_cases']]),axis=0)+4
        self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
        self.report.update(continuous_forward_inverse_trial=True,inverse90_ready=False,physical_roundtrip_verified=False,keep_cube_between_legs=True,
            placement_orientation='GREEN up; ORANGE wristward')
YawRoundtripPlan.CAPTURE_PROTOCOL_TYPE=YawRoundtripCapture;YawRoundtripPlan.CAMERA_TYPE=PairGraspCamera
def main():
    try:validate_review(YawRoundtripPlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:print('NOT READY:',exc,'. No devices opened.');return 2
    print('빈손 첫 SPACE. PLACE에서 초록 위/주황 면 손목 ID41 쪽. 손을 빼고 두 번째 SPACE.',flush=True)
    print('정회전638 -> 고정 -> 역회전638 -> 고정 -> 자동 OPEN. 두 회전 사이 큐브를 만지지 마세요.',flush=True)
    print('REMOVE에서 큐브를 뺀 뒤 세 번째 SPACE로 OFF. 실제 왕복 각도는 저장 영상으로 확인합니다.',flush=True)
    print('기존450mA/PID/75ms, 2틱 및 전압/온도/추종/통신 중단 기준 유지. Q/Esc/Ctrl+C 중단.',flush=True)
    saved=runtime.run
    try:runtime.run=confirmed_run;return runtime.main(YawRoundtripPlan,bus_type=YawRoundtripBus,screener=run_screen)
    finally:runtime.run=saved
if __name__=='__main__':raise SystemExit(main())
