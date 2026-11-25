"""Operator-triggered cube experiment after successful EMPTY finish."""
import sys,time
from loaded_quarter_finish_plan_20261025 import LoadedQuarterFinishPlan,ROOT,FIXTURE,loaded_basis
from loaded_quarter_finish_gate_20261025 import validate_proof,run_screen
from quarter_finish_empty_camera_20261025 import QuarterFinishEmptyCamera
from loaded_quarter_finish_protocol_20261025 import LoadedQuarterFinishCapture
from leap_quarter_finish_empty_20261025 import EmptyFinishBus
from loaded_quarter_reentry_20261025 import cmc_native_min
from book_grasp_trial_plan import RING_RELEASE
from quarter_current_runtime_20261025 import main
from leap_quarter_finish_empty_preview_fixed_20261025 import require_camera_mode_review
class LoadedFinishCamera(QuarterFinishEmptyCamera):EMPTY_ONLY=False
class LoadedFinishBus(EmptyFinishBus):
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
            if state is None or time.monotonic()-self.latest_release_read_at>.2:raise RuntimeError('No fresh measured thumb MCP release; no next target')
            # ID2 now begins its remaining opening during the release. Require
            # its measured position to follow that accepted goal before LIFT.
            limit=int(self.last_goal_command[1])+20
            if int(state['position'][1])>limit:raise RuntimeError('Thumb MCP ID2 has not followed the opening goal within20 ticks; no next target')
            self.report.update(thumb_mcp_release_measured_verified=True,thumb_mcp_release_actual_ticks=int(state['position'][1]),thumb_mcp_release_limit_ticks=limit)
        return super().write(addr,size,values,verify=verify)
LoadedQuarterFinishPlan.CAMERA_TYPE=LoadedFinishCamera
LoadedQuarterFinishPlan.CAPTURE_PROTOCOL_TYPE=LoadedQuarterFinishCapture
def require_offline_review():
    require_camera_mode_review();loaded_basis();return validate_proof(LoadedQuarterFinishPlan(FIXTURE/'request.json'),FIXTURE/'screen.json')
if __name__=='__main__':
    if '--execute' in sys.argv:
        try:require_offline_review()
        except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:
            print('OFFLINE REVIEW NOT READY:',str(exc),'. No devices opened.',flush=True);raise SystemExit(2)
    print('FIRST SPACE with EMPTY hand; Cube outside the hand and camera. Startup preview wait is automatic.',flush=True)
    print('Only at PLACE: GREEN UP, ORANGE toward WRIST/ID41. SECOND SPACE runs reference plus NEW finish.',flush=True)
    print('REMOVE CUBE before THIRD SPACE/OFF. Q/Esc/Ctrl+C stops. No automatic retry.',flush=True)
    raise SystemExit(main(LoadedQuarterFinishPlan,bus_type=LoadedFinishBus,screener=run_screen))
