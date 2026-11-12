"""User-started pitch/transfer observation. Default opens no devices."""
import numpy as np
from pitch_handoff_plan_20261028 import PitchHandoffPlan,REQUEST,PROOF,PITCH_HOLD,TRANSFER_HOLD
from pitch_handoff_gate_20261028 import validate_review,run_screen
from pitch_handoff_capture_20261028 import PitchHandoffCapture
from leap_calibrated_quarter_20261028 import CalibratedQuarterBus,PairGraspCamera,runtime,confirmed_run
from calibrated_two_full_health_20261028 import sample as precise_sample

def sample(bus,command,report,phase,**kwargs):
    state,errors=precise_sample(bus,command,report,phase,**kwargs)
    if hasattr(bus,'grasp_protocol'):
        if phase==PITCH_HOLD:bus.grasp_protocol.hold_frame(bus,command,report,state)
        if phase==TRANSFER_HOLD:bus.grasp_protocol.rotation_frame(bus,command,report,state)
    return state,errors

class PitchHandoffBus(CalibratedQuarterBus):
    checked_sample=staticmethod(sample)
    def approve(self,plan,proof):
        self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start
        self.screen_approved=True;self.support_checked=False
        self.root_release_threshold=np.max(np.asarray([r['root_clear_goal_ticks'] for r in self.proof['empty_recovery']['static_cases']]),axis=0)+4
        self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']

PitchHandoffPlan.CAPTURE_PROTOCOL_TYPE=PitchHandoffCapture
PitchHandoffPlan.CAMERA_TYPE=PairGraspCamera

def main():
    try:validate_review(PitchHandoffPlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:
        print('NOT READY:',exc,'. No devices opened.',flush=True);return 2
    print('FIRST SPACE in TERMINAL with EMPTY hand. Cube stays outside hand AND camera until PLACE.',flush=True)
    print('PLACE: same known pitch setup; GREEN up / ORANGE toward WRIST ID41. SECOND SPACE starts pitch then pose transfer.',flush=True)
    print('No horizontal turn follows. REMOVE cube before THIRD SPACE = OFF. Q/Esc/Ctrl+C stops.',flush=True)
    print('Same1Mbps/QPC/two-full-read recovery,450mA/PID/current/voltage/tracking stops. No live SAC.',flush=True)
    saved=runtime.run
    try:
        runtime.run=confirmed_run
        return runtime.main(PitchHandoffPlan,bus_type=PitchHandoffBus,screener=run_screen)
    finally:runtime.run=saved

if __name__=='__main__':raise SystemExit(main())
