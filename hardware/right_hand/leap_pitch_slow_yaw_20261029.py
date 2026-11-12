"""Reviewed user-started combined experiment. Default never opens devices."""
import numpy as np
from pitch_slow_yaw_plan_20261029 import PitchSlowYawPlan,REQUEST,PROOF
from pitch_handoff_plan_20261028 import PITCH_HOLD,TRANSFER_HOLD
from pitch_slow_yaw_gate_20261029 import validate_review,run_screen
from pitch_slow_yaw_capture_20261029 import PitchSlowYawCapture
from leap_calibrated_quarter_20261028 import CalibratedQuarterBus,PairGraspCamera,runtime,confirmed_run
from calibrated_two_full_health_20261028 import sample as precise_sample

def sample(bus,command,report,phase,**kwargs):
    state,errors=precise_sample(bus,command,report,phase,**kwargs)
    if hasattr(bus,'grasp_protocol'):
        if phase==PITCH_HOLD:bus.grasp_protocol.pitch_frame(bus,command,report,state)
        if phase==TRANSFER_HOLD:bus.grasp_protocol.transfer_frame(bus,command,report,state)
    return state,errors

class CombinedBus(CalibratedQuarterBus):
    checked_sample=staticmethod(sample)
    def approve(self,plan,proof):
        self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start
        self.screen_approved=True;self.support_checked=False
        self.root_release_threshold=np.max(np.asarray([r['root_clear_goal_ticks'] for r in self.proof['empty_recovery']['static_cases']]),axis=0)+4
        self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']

PitchSlowYawPlan.CAPTURE_PROTOCOL_TYPE=PitchSlowYawCapture
PitchSlowYawPlan.CAMERA_TYPE=PairGraspCamera

def main():
    try:validate_review(PitchSlowYawPlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:
        print('NOT READY:',exc,'. No devices opened.',flush=True);return 2
    print('통합 회전 실험: 빈손 준비 -> 큐브 놓기 -> 뒤집기 -> 자세 이동 -> 느린 수평 회전 -> 손 펴기 -> 큐브 제거/OFF.',flush=True)
    print('첫 SPACE는 빈손으로 누릅니다. PLACE가 뜬 뒤 초록 위/주황 손목 ID41 쪽으로 놓고 손을 뺀 뒤 두 번째 SPACE.',flush=True)
    print('중간에 큐브를 다시 놓지 않습니다. 약7~8분. REMOVE가 뜨면 큐브를 빼고 세 번째 SPACE. Q/Esc/Ctrl+C는 중단.',flush=True)
    print('수평 목표는 기존 값이며 각 목표를150ms 유지합니다.450mA와 기존PID/전압/온도/추종 기준을 유지합니다.',flush=True)
    print('이 통합 파일의 총 구동 시간 제한은480초입니다. 기존 단일 동작 파일의300초 제한은 그대로입니다.',flush=True)
    print('기본 모델 조건에서 연결 동작을 확인한 실험용입니다. 넓은 구동·마찰 조건에서는 실패도 있어 실제 결과는 저장 영상으로 확인합니다.',flush=True)
    saved=runtime.run
    try:
        runtime.run=confirmed_run
        return runtime.main(PitchSlowYawPlan,bus_type=CombinedBus,screener=run_screen)
    finally:runtime.run=saved

if __name__=='__main__':raise SystemExit(main())
