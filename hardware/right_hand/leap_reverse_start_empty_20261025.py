"""User-triggered whole EMPTY entry into the reverse-turn placement posture."""
import sys,time
from reverse_start_empty_plan_20261025 import ReverseStartPlan,ROOT,RING_RELEASE,THUMB_RELEASE
from reverse_start_empty_gate_20261025 import run_screen,validate_proof
from leap_book_cube_grasp_trial import BookGraspBus,Camera
from leap_thumb_prepare_current_empty import main

class ReverseStartBus(BookGraspBus):
    def approve(self,plan,proof):
        self.proof=validate_proof(plan,proof);self.plan_module=plan;self.planner=plan.validate_start
        self.screen_approved=True;self._goal_writes=0
        rows=plan.plan(plan.source_position())
        self._ring_release_writes=sum(p==RING_RELEASE for p,q in rows)
        self._release_writes=sum(p in (RING_RELEASE,THUMB_RELEASE) for p,q in rows)
        self._release_threshold=min(s['thumb_clear_goal_ticks'] for s in self.proof['static_screens'])-4
        self._ring_release_threshold=min(s['ring_clear_goal_ticks'] for s in self.proof['static_screens'])-4

ReverseStartPlan.CAMERA_TYPE=Camera

def require_offline_review():
    folder=ROOT/'right_hand_rotation_20261024/reverse_empty_entry/fixture_v2'
    return validate_proof(ReverseStartPlan(folder/'request.json'),folder/'screen.json')

if __name__=='__main__':
    if '--execute' in sys.argv:
        try:require_offline_review()
        except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:
            print('OFFLINE REVIEW NOT READY: '+str(exc)+'. No camera or motor port opened.',flush=True)
            raise SystemExit(2)
    print('EMPTY FOR THE ENTIRE RUN. Remove cube from hand and camera view.',flush=True)
    print('This reaches the NEW starting posture for a forward quarter turn. No cube turn is sent.',flush=True)
    print('Leave fingers, book and base as they are. One SPACE in TERMINAL starts fresh checks.',flush=True)
    print('Camera opens automatically. Same current/PID and tracking/voltage stops. OFF at completion.',flush=True)
    raise SystemExit(main(ReverseStartPlan,bus_type=ReverseStartBus,screener=run_screen))
