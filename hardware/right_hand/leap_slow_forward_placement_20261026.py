"""Operator-triggered75ms pacing experiment. Voltage floor is unchanged."""
import sys
from slow_forward_plan_20261026 import SlowForwardPlan,ROOT,FIXTURE
from slow_forward_gate_20261026 import validate_proof,run_screen
from forward_cube_placement_20261026 import require_placement_review
from leap_recovered_forward_placement_20261026 import RecoveredForwardBus
from thumb_mcp_clearance_guard_20261026 import policy as mcp_clearance_policy
from loaded_quarter_reentry_20261025 import cmc_native_min
from book_grasp_trial_plan import RING_RELEASE
from slow_forward_current_runtime_20261026 import main
class SlowForwardBus(RecoveredForwardBus):
    def approve(self,plan,proof):
        self.proof=validate_proof(plan,proof);self.plan_module=plan;self.planner=plan.validate_start
        self.screen_approved=True;self._goal_writes=0;rows=plan.plan(plan.source_position())
        self._ring_release_writes=sum(p==RING_RELEASE for p,q in rows)
        self._ring_release_threshold=min(s['ring_clear_goal_ticks'] for s in self.proof['static_screens'])-4
        self._tip_release_threshold=min(s['thumb_tip_clear_goal_ticks'] for s in self.proof['static_screens'])-4
        self._cmc_native_reentry_required=int(plan.source_position()[3])<cmc_native_min()
        self._mcp_release_policy=mcp_clearance_policy(plan,self.proof)
def require_review():
    require_placement_review()
    return validate_proof(SlowForwardPlan(FIXTURE/'request.json'),FIXTURE/'screen.json')
if __name__=='__main__':
    if '--execute' in sys.argv:
        try:require_review()
        except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:
            print('SLOW REVIEW NOT READY:',str(exc),'. No devices opened.',flush=True);raise SystemExit(2)
    print('SLOW1.5x duration. Same goal positions, PID/current and voltage stops. Power wiring unchanged.',flush=True)
    print('EMPTY hand at FIRST SPACE. Cube only at PLACE. REMOVE before THIRD SPACE/OFF.',flush=True)
    print('This pacing experiment does not guarantee a voltage improvement or physical flip.',flush=True)
    raise SystemExit(main(SlowForwardPlan,bus_type=SlowForwardBus,screener=run_screen))
