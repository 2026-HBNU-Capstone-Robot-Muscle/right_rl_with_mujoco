"""Operator-started EMPTY comparison with bounded ID12 catch-up waits."""
import numpy as np
from adaptive_yaw_empty_plan_20261029 import AdaptiveYawEmptyPlan,REQUEST,PROOF
from adaptive_yaw_empty_gate_20261029 import validate_review,run_screen
from leap_yaw_load_empty_20261029 import EmptyPrefixBus,EmptyPrefixProtocol,EmptyCamera,runtime
from root_lag_pause_20261029 import RootLagPause,POLICY
from root_lag_motion_runner_20261029 import run
class AdaptiveEmptyBus(EmptyPrefixBus):
    def approve(self,plan,proof):
        self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start;self.screen_approved=True;self.support_checked=False
        self.root_release_threshold=np.max(np.asarray([v['root_clear_goal_ticks'] for v in self.proof['empty_recovery']['static_cases']]),axis=0)+4
        self.initial_pair_checked=not self.proof['initial_self_separation']['enabled'];self.root_lag_pause=RootLagPause()
        self.report.update(cube_present=False,diagnostic_condition='EMPTY throughout; bounded catch-up timing',root_lag_pause_policy=dict(POLICY))
AdaptiveYawEmptyPlan.CAPTURE_PROTOCOL_TYPE=EmptyPrefixProtocol;AdaptiveYawEmptyPlan.CAMERA_TYPE=EmptyCamera
def main():
    try:validate_review(AdaptiveYawEmptyPlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError,AssertionError) as exc:print('NOT READY:',exc,'. No devices opened.');return 2
    print('끝까지 빈손. 큐브를 손과 카메라 시야 밖에 두고 터미널 SPACE 한 번만 누릅니다. 자동 OFF합니다.',flush=True)
    print('같은 관절 목표입니다. 회전 중ID12가0.12rad 뒤처지면 새 목표를 보내지 않고 기다립니다.',flush=True)
    print('0.10rad 이내 정상 상태2회를 확인하면 계속합니다. 대기당최대1.2초/총20초이며 기존300초에 포함됩니다.',flush=True)
    print('기존전류/PID/0.18rad 추종중단/전압/온도/진행검사를 유지합니다. PLACE나추가SPACE는없습니다. Q로중단.',flush=True)
    saved=runtime.run
    try:runtime.run=run;return runtime.main(AdaptiveYawEmptyPlan,bus_type=AdaptiveEmptyBus,screener=run_screen)
    finally:runtime.run=saved
if __name__=='__main__':raise SystemExit(main())
