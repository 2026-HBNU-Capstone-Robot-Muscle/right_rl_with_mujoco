"""User-triggered placement observation at the physically reached new start."""
import sys,time
from reverse_place_capture_plan_20261025 import ReversePlacePlan,ROOT,BASE_RELEASE,CASING_PAIR
from reverse_place_capture_protocol_20261025 import ReversePlacementCapture
from reverse_place_capture_gate_20261025 import run_screen,validate_proof
from leap_reverse_start_empty_20261025 import ReverseStartBus
from leap_book_cube_grasp_trial import Camera
from leap_thumb_prepare_current_empty import main

class PlacementBus(ReverseStartBus):
    def approve(self,plan,proof):
        self.proof=validate_proof(plan,proof);self.plan_module=plan;self.planner=plan.validate_start
        self.screen_approved=True;self._goal_writes=0;rows=plan.plan(plan.source_position())
        cases=[s for s in self.proof['static_screens'] if CASING_PAIR in s['initial_self_contacts_mm']]
        if cases:
            self._base_release_writes=sum(p==BASE_RELEASE for p,q in rows)
            self._base_release_threshold=min(s['thumb_clear_goal_ticks'] for s in cases)-4

    def write(self,addr,size,values,verify=True):
        if addr==116 and hasattr(self,'_base_release_writes') and self._goal_writes==2+self._base_release_writes:
            state=getattr(self,'latest_release_state',None)
            if state is None or time.monotonic()-self.latest_release_read_at>.2:
                raise RuntimeError('No fresh thumb-base release measurement; no root target advance')
            if int(state['position'][3])>self._base_release_threshold:
                raise RuntimeError('Thumb-base release not measured; no root target advance')
            self.report['thumb_base_release_measured_verified']=True
            self.report['thumb_base_release_actual_ticks']=int(state['position'][3])
        return super().write(addr,size,values,verify=verify)

ReversePlacePlan.CAMERA_TYPE=Camera
ReversePlacePlan.CAPTURE_PROTOCOL_TYPE=ReversePlacementCapture

def require_offline_review():
    folder=ROOT/'right_hand_rotation_20261024/reverse_empty_entry/placement_fixture_v8'
    return validate_proof(ReversePlacePlan(folder/'request.json'),folder/'screen.json')

if __name__=='__main__':
    if '--execute' in sys.argv:
        try:require_offline_review()
        except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:
            print('OFFLINE REVIEW NOT READY: '+str(exc)+'. No camera or motor port opened.',flush=True);raise SystemExit(2)
    print('Keep cube OUT for FIRST SPACE. Fresh OFF readings restore the reached start automatically.',flush=True)
    print('PLACE prompt: cube on INDEX/MIDDLE flat link tops, GREEN UP, ORANGE toward WRIST.',flush=True)
    print('SECOND SPACE = photo with all goals fixed. REMOVE cube, THIRD SPACE = OFF. No rotation.',flush=True)
    raise SystemExit(main(ReversePlacePlan,bus_type=PlacementBus,screener=run_screen))
