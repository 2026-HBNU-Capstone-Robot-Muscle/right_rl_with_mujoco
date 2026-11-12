"""Supervised cube grasp after the completed book-aware empty preparation."""
from book_grasp_trial_plan import BookGraspPlan
from book_grasp_trial_gate import run_screen,validate_proof
from leap_thumb_prepare_current_empty import Bus,main
from marker_trial_camera import MarkerTrialCamera
import sys
import time
from book_grasp_trial_plan import THUMB_RELEASE,RING_RELEASE
from book_thumb_release_guard import verify as verify_thumb_release
from book_grasp_trial_plan import ROOT


def require_offline_review():
    # Development readiness only. This old OFF snapshot never substitutes for
    # the runtime's three fresh stable OFF reads and its own full native replay.
    folder=ROOT/'right_hand_rotation_20261024/book20_current_review/grasp_route_fixture_v7'
    return validate_proof(BookGraspPlan(folder/'request.json'),folder/'screen.json')


class Camera(MarkerTrialCamera):
    EXPOSURE=-5.
    CONFIRM_PROPERTY_READS=True
    # This observation trial does not compute 3-D control from marker exposure.
    # Preserve the camera's current exposure; quality/freshness/EMPTY/Q checks
    # still use actual frames. A driver property GET is not a motion condition.
    PRESERVE_EXPOSURE=True


class BookGraspBus(Bus):
    def approve(self,plan,proof):
        self.proof=validate_proof(plan,proof)
        self.plan_module=plan;self.planner=plan.validate_start
        self.screen_approved=True
        self._goal_writes=0
        self._ring_release_writes=sum(p==RING_RELEASE for p,q in plan.plan(plan.source_position()))
        self._release_writes=sum(p in (RING_RELEASE,THUMB_RELEASE) for p,q in plan.plan(plan.source_position()))
        self._release_threshold=min(s['thumb_clear_goal_ticks'] for s in self.proof['static_screens'])-4
        self._ring_release_threshold=min(s['ring_clear_goal_ticks'] for s in self.proof['static_screens'])-4

    def snapshot(self):
        state=super().snapshot()
        self.latest_release_state=state;self.latest_release_read_at=time.monotonic()
        return state

    def write(self,addr,size,values,verify=True):
        if addr==116 and hasattr(self,'_ring_release_writes') and self._goal_writes==2+self._ring_release_writes:
            state=getattr(self,'latest_release_state',None)
            if state is None or time.monotonic()-self.latest_release_read_at>.2:
                raise RuntimeError('No fresh measured ring release; no thumb target advance')
            if int(state['position'][14])>self._ring_release_threshold:
                raise RuntimeError('Ring release not measured; no thumb target advance')
            self.report['ring_release_measured_verified']=True
        if addr==116 and hasattr(self,'_release_writes') and self._goal_writes==2+self._release_writes:
            state=getattr(self,'latest_release_state',None)
            if state is None or time.monotonic()-self.latest_release_read_at>.2:
                raise RuntimeError('No fresh measured thumb release; no root target advance')
            verify_thumb_release(state,self.plan_module.source_position(),self._release_threshold,self.report)
            if not self.report.get('ring_release_measured_verified') or int(state['position'][14])>self._ring_release_threshold:
                raise RuntimeError('Ring release not measured; no root target advance')
        result=super().write(addr,size,values,verify=verify)
        if addr==116:self._goal_writes+=1
        return result


BookGraspPlan.CAMERA_TYPE=Camera

if __name__=='__main__':
    if '--execute' in sys.argv:
        try:require_offline_review()
        except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:
            print('OFFLINE REVIEW NOT READY: '+str(exc)+'. No camera or motor port opened.',flush=True)
            raise SystemExit(2)
    print('Leave the book/base and fingers as they are. Start with the hand EMPTY.',flush=True)
    print('Observed OFF thumb sag is included in the model; recovery does not deepen it.',flush=True)
    print('FIRST SPACE prepares EMPTY. Only at PLACE CUBE put GREEN up, ORANGE toward camera.',flush=True)
    print('SECOND SPACE: hands clear, slow grasp. REMOVE CUBE then THIRD SPACE: OFF.',flush=True)
    print('One grasp observation only. No cube turn or live SAC. Q stops.',flush=True)
    raise SystemExit(main(BookGraspPlan,bus_type=BookGraspBus,screener=run_screen))

