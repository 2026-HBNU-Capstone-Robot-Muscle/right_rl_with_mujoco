"""Operator-started unchanged yaw with the completed empty-test OFF readbacks."""
import time,numpy as np
from paced_retained_yaw_plan_20261029 import PacedRetainedYawPlan,REQUEST,PROOF
from paced_retained_yaw_gate_20261029 import validate_review,run_screen,PASSIVE_POLICY
from leap_retained_slow_yaw_20261029 import RetainedCapture,PairGraspCamera,runtime,confirmed_run
from leap_empty_hold_comm_20261029 import EmptyBus
def pace_observer(observe,cancel,*,period=.075,now=time.perf_counter,sleep=time.sleep):
    last_start=None
    def paced(phase):
        nonlocal last_start
        cancel()
        if last_start is not None:
            remaining=period-(now()-last_start)
            if remaining>0:sleep(remaining)
        cancel();last_start=now()
        return observe(phase)
    return paced
class PacedCapture(RetainedCapture):
    def _observer(self,bus,command,report,guard):
        report['passive_observation_policy']=dict(PASSIVE_POLICY)
        return pace_observer(super()._observer(bus,command,report,guard),self.cancel,sleep=self.sleep)
class PacedBus(EmptyBus):
    def approve(self,plan,proof):
        self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start;self.screen_approved=True;self.support_checked=False
        self.root_release_threshold=np.max(np.asarray([v['root_clear_goal_ticks'] for v in self.proof['empty_recovery']['static_cases']]),axis=0)+4
        self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
PacedRetainedYawPlan.CAPTURE_PROTOCOL_TYPE=PacedCapture;PacedRetainedYawPlan.CAMERA_TYPE=PairGraspCamera
def main():
    try:validate_review(PacedRetainedYawPlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:print('NOT READY:',exc,'. No devices opened.');return 2
    print('빈손으로 첫 SPACE. PLACE가 뜨면 빨강 위/초록 손목 ID41 쪽으로 놓고 손을 뺀 뒤 두 번째 SPACE.',flush=True)
    print('수평 회전 후 자동 OPEN합니다. REMOVE에서 큐브를 빼고 세 번째 SPACE. Q/Esc/Ctrl+C로 중단.',flush=True)
    print('동작은 약4~5분입니다. 큐브를 놓고 빼는 시간은 별도입니다.',flush=True)
    print('배치 대기/사진/카운트다운도75ms 주기로 전체 건강 상태를 확인합니다. 기존200ms 정상 상태2회 확인을 유지합니다.',flush=True)
    print('OFF 뒤 설정 확인은 완료된 빈손 검사와 같은 전체 고속 읽기를 사용합니다. 목표/설정 쓰기 재시도는 없습니다.',flush=True)
    print('같은 느린 수평 회전 경로입니다. 실제90도 회전과 견고성은 아직 확인되지 않았습니다.',flush=True)
    saved=runtime.run
    try:runtime.run=confirmed_run;return runtime.main(PacedRetainedYawPlan,bus_type=PacedBus,screener=run_screen)
    finally:runtime.run=saved
if __name__=='__main__':raise SystemExit(main())
