"""Operator-started loaded trial; default mode opens no device."""
import sys
from loaded_thumb_rotation_plan_20261026 import LoadedThumbRotationPlan,ROOT,FIXTURE,loaded_basis
from loaded_thumb_rotation_gate_20261026 import validate_proof,run_screen
from leap_thumb_clear_root_empty_20261026 import ThumbRootEmptyBus,require_review as require_empty_review
from loaded_thumb_rotation_camera_20261026 import LoadedThumbCamera
from loaded_thumb_rotation_protocol_20261026 import LoadedThumbCapture
from loaded_thumb_rotation_runtime_20261026 import main
from loaded_thumb_voltage_sample_20261026 import sample
from loaded_thumb_rest_policy_20261026 import metadata
from thumb_mcp_clearance_guard_20261026 import policy as mcp_policy
from loaded_quarter_reentry_20261025 import cmc_native_min
from book_grasp_trial_plan import RING_RELEASE
class LoadedThumbBus(ThumbRootEmptyBus):
    checked_sample=staticmethod(sample)
    def approve(self,plan,proof):
        self.proof=validate_proof(plan,proof);self.plan_module=plan;self.planner=plan.validate_start
        self.screen_approved=True;self._goal_writes=0;rows=plan.plan(plan.source_position())
        self._ring_release_writes=sum(p==RING_RELEASE for p,q in rows)
        self._ring_release_threshold=min(s['ring_clear_goal_ticks'] for s in self.proof['static_screens'])-4
        self._tip_release_threshold=min(s['thumb_tip_clear_goal_ticks'] for s in self.proof['static_screens'])-4
        self._cmc_native_reentry_required=int(plan.source_position()[3])<cmc_native_min()
        self._mcp_release_policy=mcp_policy(plan,self.proof)
LoadedThumbRotationPlan.CAMERA_TYPE=LoadedThumbCamera
LoadedThumbRotationPlan.CAPTURE_PROTOCOL_TYPE=LoadedThumbCapture
def require_review():
    require_empty_review();loaded_basis()
    return validate_proof(LoadedThumbRotationPlan(FIXTURE/'request.json'),FIXTURE/'screen.json')
if __name__=='__main__':
    if '--execute' in sys.argv:
        print('Checking saved EMPTY and loaded reviews; no devices opened yet.',flush=True)
        try:require_review()
        except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:
            print('LOADED THUMB TRIAL NOT READY:',str(exc),'. No devices opened.',flush=True);raise SystemExit(2)
    print('FIRST SPACE EMPTY. Cube only at PLACE; GREEN UP, ORANGE toward WRIST/ID41.',flush=True)
    print('SECOND SPACE starts about96s turn with thumb retreat/root lift; REMOVE before THIRD SPACE = OFF.',flush=True)
    print('Actual90deg cube rotation is unverified. No automatic retry or live SAC. Existing voltage/current/tracking stops remain.',flush=True)
    raise SystemExit(main(LoadedThumbRotationPlan,bus_type=LoadedThumbBus,screener=run_screen))
