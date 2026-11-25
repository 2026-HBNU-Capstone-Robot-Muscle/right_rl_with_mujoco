"""Operator-triggered EMPTY whole-path check. Never executes on import."""
import sys
from reverse_quarter_path_empty_plan_20261025 import ReverseQuarterEmptyPlan,ROOT,BASE_RELEASE,CASING_PAIR
from reverse_quarter_path_empty_gate_20261025 import run_screen,validate_proof
from leap_reverse_place_capture_20261025 import PlacementBus
from leap_book_cube_grasp_trial import Camera
from leap_thumb_prepare_current_empty import main

class QuarterEmptyBus(PlacementBus):
    def approve(self,plan,proof):
        self.proof=validate_proof(plan,proof);self.plan_module=plan;self.planner=plan.validate_start
        self.screen_approved=True;self._goal_writes=0;rows=plan.plan(plan.source_position())
        cases=[s for s in self.proof['static_screens'] if CASING_PAIR in s['initial_self_contacts_mm']]
        if cases:
            self._base_release_writes=sum(p==BASE_RELEASE for p,q in rows)
            self._base_release_threshold=min(s['thumb_clear_goal_ticks'] for s in cases)-4

ReverseQuarterEmptyPlan.CAMERA_TYPE=Camera

def require_offline_review():
    folder=ROOT/'right_hand_rotation_20261024/reverse_quarter_path_empty/fixture_v1'
    return validate_proof(ReverseQuarterEmptyPlan(folder/'request.json'),folder/'screen.json')

if __name__=='__main__':
    if '--execute' in sys.argv:
        try:require_offline_review()
        except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:
            print('OFFLINE REVIEW NOT READY: '+str(exc)+'. No camera or motor port opened.',flush=True);raise SystemExit(2)
    print('EMPTY FOR THE ENTIRE RUN. Cube OUT of hand and camera view. No PLACE prompt.',flush=True)
    print('Fresh OFF recovery, then the WHOLE proposed quarter-turn hand path once, then automatic OFF.',flush=True)
    print('One SPACE in TERMINAL starts fresh checks. Q/Esc/Ctrl+C stops. Camera opens automatically.',flush=True)
    print('Same current/PID, <=2-tick steps, and actual tracking/voltage/current stops. No SAC or physical Cube turn.',flush=True)
    raise SystemExit(main(ReverseQuarterEmptyPlan,bus_type=QuarterEmptyBus,screener=run_screen))
