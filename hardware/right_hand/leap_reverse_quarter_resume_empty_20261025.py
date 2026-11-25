"""Supervised EMPTY suffix; activation still requires the user's terminal SPACE."""
import sys
from reverse_quarter_resume_empty_plan_20261025 import ReverseQuarterResumeEmptyPlan,ROOT
from reverse_quarter_resume_empty_gate_20261025 import validate_proof,run_screen
from book_grasp_trial_plan import RING_RELEASE
from leap_reverse_quarter_path_empty_20261025 import QuarterEmptyBus
from leap_book_cube_grasp_trial import Camera
from quarter_current_runtime_20261025 import main

class ResumeBus(QuarterEmptyBus):
    def approve(self,plan,proof):
        self.proof=validate_proof(plan,proof);self.plan_module=plan;self.planner=plan.validate_start
        self.screen_approved=True;self._goal_writes=0;rows=plan.plan(plan.source_position())
        self._ring_release_writes=sum(p==RING_RELEASE for p,q in rows)
        self._ring_release_threshold=min(s['ring_clear_goal_ticks'] for s in self.proof['static_screens'])-4

ReverseQuarterResumeEmptyPlan.CAMERA_TYPE=Camera

def require_offline_review():
    folder=ROOT/'right_hand_rotation_20261024/reverse_quarter_resume_empty/fixture_v4'
    return validate_proof(ReverseQuarterResumeEmptyPlan(folder/'request.json'),folder/'screen.json')

if __name__=='__main__':
    if '--execute' in sys.argv:
        try:require_offline_review()
        except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:
            print('OFFLINE REVIEW NOT READY: '+str(exc)+'. No camera or motor port opened.',flush=True);raise SystemExit(2)
    print('CUBE OUT for the entire run. One terminal SPACE starts fresh OFF checks, recovery, and ONLY the remaining reference.',flush=True)
    print('The earlier 23.6s reference prefix is not repeated. Camera opens automatically. Q/Esc/Ctrl+C stops.',flush=True)
    print('Goal-aware progress monitoring; unchanged actual voltage/current/tracking stops. Automatic OFF.',flush=True)
    raise SystemExit(main(ReverseQuarterResumeEmptyPlan,bus_type=ResumeBus,screener=run_screen))
