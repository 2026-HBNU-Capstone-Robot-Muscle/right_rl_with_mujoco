"""User-triggered EMPTY continuation test; hardware only after --execute/SPACE."""
import sys
from pathlib import Path
from quarter_finish_empty_plan_20261025 import QuarterFinishEmptyPlan,ROOT,FIXTURE,empty_basis
from quarter_finish_empty_gate_20261025 import validate_proof,run_screen
from leap_loaded_quarter_trial_20261025 import LoadedQuarterBus
from loaded_quarter_reentry_20261025 import cmc_native_min
from book_grasp_trial_plan import RING_RELEASE
from empty_trial_camera import EmptyTrialCamera
from quarter_current_runtime_20261025 import main
class EmptyFinishBus(LoadedQuarterBus):
    def approve(self,plan,proof):
        self.proof=validate_proof(plan,proof);self.plan_module=plan;self.planner=plan.validate_start
        self.screen_approved=True;self._goal_writes=0;rows=plan.plan(plan.source_position())
        self._ring_release_writes=sum(p==RING_RELEASE for p,q in rows)
        self._ring_release_threshold=min(s['ring_clear_goal_ticks'] for s in self.proof['static_screens'])-4
        self._tip_release_threshold=min(s['thumb_tip_clear_goal_ticks'] for s in self.proof['static_screens'])-4
        self._cmc_native_reentry_required=int(plan.source_position()[3])<cmc_native_min()
QuarterFinishEmptyPlan.CAMERA_TYPE=EmptyTrialCamera
def require_offline_review():
    empty_basis();return validate_proof(QuarterFinishEmptyPlan(FIXTURE/'request.json'),FIXTURE/'screen.json')
if __name__=='__main__':
    if '--execute' in sys.argv:
        try:require_offline_review()
        except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:
            print('OFFLINE REVIEW NOT READY:',str(exc),'. No devices opened.',flush=True);raise SystemExit(2)
    print('EMPTY for the ENTIRE run. Cube outside hand AND camera. No PLACE prompt.',flush=True)
    print('Known reference plus NEW 3.2s root finish. One terminal SPACE; Q/Esc/Ctrl+C stops.',flush=True)
    raise SystemExit(main(QuarterFinishEmptyPlan,bus_type=EmptyFinishBus,screener=run_screen))
