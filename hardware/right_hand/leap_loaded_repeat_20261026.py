"""Operator-started repeat; same motor path and force limits as the successful run."""
import sys
from loaded_repeat_plan_20261026 import LoadedRepeatPlan,ROOT,FIXTURE
from loaded_repeat_gate_20261026 import validate_proof,run_screen
from leap_loaded_thumb_rotation_20261026 import LoadedThumbBus,require_review as require_previous_review
from loaded_thumb_rotation_camera_20261026 import LoadedThumbCamera
from loaded_thumb_rotation_protocol_20261026 import LoadedThumbCapture
from loaded_thumb_rotation_runtime_20261026 import main
from loaded_repeat_voltage_20261026 import sample
from thumb_mcp_clearance_guard_20261026 import policy as mcp_policy
from loaded_quarter_reentry_20261025 import cmc_native_min
from book_grasp_trial_plan import RING_RELEASE
class LoadedRepeatBus(LoadedThumbBus):
    checked_sample=staticmethod(sample)
    def approve(self,plan,proof):
        self.proof=validate_proof(plan,proof);self.plan_module=plan;self.planner=plan.validate_start
        self.screen_approved=True;self._goal_writes=0;rows=plan.plan(plan.source_position())
        self._ring_release_writes=sum(p==RING_RELEASE for p,q in rows)
        self._ring_release_threshold=min(s['ring_clear_goal_ticks'] for s in self.proof['static_screens'])-4
        self._tip_release_threshold=min(s['thumb_tip_clear_goal_ticks'] for s in self.proof['static_screens'])-4
        self._cmc_native_reentry_required=int(plan.source_position()[3])<cmc_native_min()
        self._mcp_release_policy=mcp_policy(plan,self.proof)
LoadedRepeatPlan.CAMERA_TYPE=LoadedThumbCamera
LoadedRepeatPlan.CAPTURE_PROTOCOL_TYPE=LoadedThumbCapture
def require_review():
    require_previous_review()
    return validate_proof(LoadedRepeatPlan(FIXTURE/'request.json'),FIXTURE/'screen.json')
if __name__=='__main__':
    if '--execute' in sys.argv:
        print('Checking saved review chain first. No devices opened yet.',flush=True)
        try:require_review()
        except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:
            print('REPEAT NOT READY:',str(exc),'. No devices opened.',flush=True);raise SystemExit(2)
    print('FIRST SPACE EMPTY. Cube only at PLACE: GREEN UP, ORANGE toward WRIST/ID41.',flush=True)
    print('SECOND SPACE runs the same about96s motion. REMOVE cube before THIRD SPACE.',flush=True)
    print('Voltage: same150ms/two >=3.8V reads/<=3.5V immediate stop; precise clock,10ms minimum polling cycle INCLUDING read time.',flush=True)
    raise SystemExit(main(LoadedRepeatPlan,bus_type=LoadedRepeatBus,screener=run_screen))
