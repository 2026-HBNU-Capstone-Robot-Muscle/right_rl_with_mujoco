"""User-triggered EMPTY proof run of the new thumb/root motion."""
import sys
from thumb_clear_root_empty_plan_20261026 import ThumbRootEmptyPlan,ROOT,FIXTURE
from thumb_clear_root_empty_gate_20261026 import validate_proof,run_screen
from leap_slow_forward_placement_20261026 import SlowForwardBus
from leap_recovered_forward_placement_20261026 import require_review as require_parent_review
from thumb_mcp_clearance_guard_20261026 import policy as mcp_policy
from loaded_quarter_reentry_20261025 import cmc_native_min
from book_grasp_trial_plan import RING_RELEASE
from quarter_finish_empty_camera_20261025 import QuarterFinishEmptyCamera
from slow_forward_current_runtime_20261026 import main

class ThumbRootEmptyBus(SlowForwardBus):
    def approve(self,plan,proof):
        self.proof=validate_proof(plan,proof);self.plan_module=plan;self.planner=plan.validate_start
        self.screen_approved=True;self._goal_writes=0;rows=plan.plan(plan.source_position())
        self._ring_release_writes=sum(p==RING_RELEASE for p,q in rows)
        self._ring_release_threshold=min(s['ring_clear_goal_ticks'] for s in self.proof['static_screens'])-4
        self._tip_release_threshold=min(s['thumb_tip_clear_goal_ticks'] for s in self.proof['static_screens'])-4
        self._cmc_native_reentry_required=int(plan.source_position()[3])<cmc_native_min()
        self._mcp_release_policy=mcp_policy(plan,self.proof)
ThumbRootEmptyPlan.CAMERA_TYPE=QuarterFinishEmptyCamera
def require_review():
    require_parent_review()
    return validate_proof(ThumbRootEmptyPlan(FIXTURE/'request.json'),FIXTURE/'screen.json')
if __name__=='__main__':
    if '--execute' in sys.argv:
        try:require_review()
        except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:
            print('EMPTY PROPOSAL NOT READY:',str(exc),'. No devices opened.',flush=True);raise SystemExit(2)
    print('EMPTY throughout: keep cube outside hand AND camera. There is NO PLACE step.',flush=True)
    print('New ending: thumb first, then index/middle roots +33.75deg.75ms cycles; current/PID/voltage/tracking stops unchanged.',flush=True)
    raise SystemExit(main(ThumbRootEmptyPlan,bus_type=ThumbRootEmptyBus,screener=run_screen))
