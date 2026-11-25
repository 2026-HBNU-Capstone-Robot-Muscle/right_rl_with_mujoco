"""One operator-triggered comparison; same motion/health and PLACE/REMOVE protocol."""
import sys,json
from recovered_forward_plan_20261026 import RecoveredForwardPlan,ROOT,FIXTURE
from recovered_forward_gate_20261026 import validate_proof,run_screen
from forward_cube_placement_20261026 import require_placement_review
from leap_loaded_quarter_finish_trial_20261025 import LoadedFinishBus
from loaded_quarter_reentry_20261025 import cmc_native_min
from book_grasp_trial_plan import RING_RELEASE
from quarter_current_runtime_20261025 import main
class RecoveredForwardBus(LoadedFinishBus):
    def approve(self,plan,proof):
        self.proof=validate_proof(plan,proof);self.plan_module=plan;self.planner=plan.validate_start
        self.screen_approved=True;self._goal_writes=0;rows=plan.plan(plan.source_position())
        self._ring_release_writes=sum(p==RING_RELEASE for p,q in rows)
        self._ring_release_threshold=min(s['ring_clear_goal_ticks'] for s in self.proof['static_screens'])-4
        self._tip_release_threshold=min(s['thumb_tip_clear_goal_ticks'] for s in self.proof['static_screens'])-4
        self._cmc_native_reentry_required=int(plan.source_position()[3])<cmc_native_min()
def require_review():
    require_placement_review()
    return validate_proof(RecoveredForwardPlan(FIXTURE/'request.json'),FIXTURE/'screen.json')
if __name__=='__main__':
    if '--execute' in sys.argv:
        try:require_review()
        except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:
            print('RECOVERY REVIEW NOT READY:',str(exc),'. No devices opened.',flush=True);raise SystemExit(2)
    print('EMPTY at FIRST SPACE. Current OFF thumb contact is checked during separation; same turn goals.',flush=True)
    print('Cube only at PLACE: about5mm toward tips. GREEN UP, ORANGE toward wrist. REMOVE before THIRD SPACE.',flush=True)
    print('If camera goes BLACK, Q/stop; do not repeatedly rerun. Camera/health stops remain active.',flush=True)
    raise SystemExit(main(RecoveredForwardPlan,bus_type=RecoveredForwardBus,screener=run_screen))
