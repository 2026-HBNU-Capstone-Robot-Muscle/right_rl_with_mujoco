"""User-started shorter-route experiment. Default opens no devices."""
import numpy as np
from calibrated_quarter_plan_20261028 import HorizontalQuarterPlan,REQUEST,PROOF
from calibrated_quarter_gate_20261028 import validate_review,run_screen
from leap_horizontal_quarter_20261027 import HorizontalQuarterBus,HorizontalQuarterCapture,PairGraspCamera,runtime,confirmed_run
class CalibratedQuarterBus(HorizontalQuarterBus):
    def approve(self,plan,proof):
        self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start;self.screen_approved=True;self.support_checked=False
        self.root_release_threshold=np.max(np.array([row['root_clear_goal_ticks'] for row in self.proof['empty_recovery']['static_cases']]),axis=0)+4
        self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
HorizontalQuarterPlan.CAPTURE_PROTOCOL_TYPE=HorizontalQuarterCapture
HorizontalQuarterPlan.CAMERA_TYPE=PairGraspCamera
def operator_main():
    previous=runtime.run
    try:
        runtime.run=confirmed_run
        return runtime.main(HorizontalQuarterPlan,bus_type=CalibratedQuarterBus,screener=run_screen)
    finally:runtime.run=previous
def main():
    try:validate_review(HorizontalQuarterPlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:print('NOT READY:',exc,'. No devices opened.',flush=True);return 2
    print('EMPTY at FIRST SPACE. PLACE only at prompt: GREEN up, ORANGE toward WRIST/ID41.',flush=True)
    print('SECOND SPACE: same grasp, shorter638-goal turn,3s HOLD, automatic OPEN. REMOVE before THIRD SPACE = OFF.',flush=True)
    print('Timing calibrated to an observed~90deg frame. Shorter-route physical angle is unverified; no live SAC.',flush=True)
    print('Same verified1Mbps,450mA/PID,2-tick steps and all health stops. Q/Esc/Ctrl+C stops.',flush=True)
    return operator_main()
if __name__=='__main__':raise SystemExit(main())
