"""Operator-run full experimental reference. No hardware on import/default."""
import sys
import time
from loaded_quarter_trial_plan_20261025 import LoadedQuarterTrialPlan,ROOT,validate_experiment_basis
from loaded_quarter_trial_gate_20261025 import validate_proof,run_screen
from loaded_quarter_trial_protocol_20261025 import LoadedQuarterCapture
from book_grasp_trial_plan import RING_RELEASE
from leap_reverse_quarter_path_empty_20261025 import QuarterEmptyBus
from leap_book_cube_grasp_trial import Camera
from quarter_current_runtime_20261025 import main
from loaded_quarter_reentry_20261025 import cmc_native_min

class LoadedQuarterBus(QuarterEmptyBus):
    def approve(self,plan,proof):
        self.proof=validate_proof(plan,proof);self.plan_module=plan;self.planner=plan.validate_start
        self.screen_approved=True;self._goal_writes=0;rows=plan.plan(plan.source_position())
        self._ring_release_writes=sum(p==RING_RELEASE for p,q in rows)
        self._ring_release_threshold=min(s['ring_clear_goal_ticks'] for s in self.proof['static_screens'])-4
        self._tip_release_threshold=min(s['thumb_tip_clear_goal_ticks'] for s in self.proof['static_screens'])-4
        self._cmc_native_reentry_required=int(plan.source_position()[3])<cmc_native_min()

    def write(self,addr,size,values,verify=True):
        if addr==116 and hasattr(self,'_ring_release_writes') and self._goal_writes==2+self._ring_release_writes:
            state=getattr(self,'latest_release_state',None)
            if state is None or time.monotonic()-self.latest_release_read_at>.2:raise RuntimeError('No fresh measured thumb-tip release; no next target')
            if self._cmc_native_reentry_required and int(state['position'][3])<cmc_native_min():raise RuntimeError('Thumb-base ID4 has not re-entered its native range; no next target')
            if int(state['position'][0])>self._tip_release_threshold:raise RuntimeError('Thumb-tip release not measured; no next target')
            self.report['thumb_tip_release_measured_verified']=True
        return super().write(addr,size,values,verify=verify)

LoadedQuarterTrialPlan.CAMERA_TYPE=Camera
LoadedQuarterTrialPlan.CAPTURE_PROTOCOL_TYPE=LoadedQuarterCapture

def require_offline_review():
    validate_experiment_basis();folder=ROOT/'right_hand_rotation_20261024/loaded_quarter_trial_20261025/fixture_v6'
    return validate_proof(LoadedQuarterTrialPlan(folder/'request.json'),folder/'screen.json')

if __name__=='__main__':
    if '--execute' in sys.argv:
        try:require_offline_review()
        except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:
            print('OFFLINE REVIEW NOT READY: '+str(exc)+'. No devices opened.',flush=True);raise SystemExit(2)
    print('FIRST SPACE with EMPTY hand. Camera opens automatically; no need to reshape fingers.',flush=True)
    print('Only at PLACE: green up, orange toward WRIST/ID41. SECOND SPACE starts full reference, not a small turn.',flush=True)
    print('REMOVE cube before THIRD SPACE = OFF. Q/Esc/Ctrl+C stops; no automatic retry.',flush=True)
    raise SystemExit(main(LoadedQuarterTrialPlan,bus_type=LoadedQuarterBus,screener=run_screen))
