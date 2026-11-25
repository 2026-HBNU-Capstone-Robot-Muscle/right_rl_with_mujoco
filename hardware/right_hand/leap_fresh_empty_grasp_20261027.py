"""Operator-controlled EMPTY posture experiment. Default opens no device."""
import sys
import numpy as np
from fresh_empty_grasp_plan_20261027 import FreshEmptyGraspPlan,REQUEST,PROOF,ROOT_PHASE
from fresh_empty_grasp_gate_20261027 import validate_proof,run_screen
from leap_thumb_prepare_current_empty import Bus as GuardedBus
from recovering_empty_camera_20261026 import RecoveringEmptyCamera
from camera_gap_current_runtime_20261026 import main
from empty_precise_voltage_20261027 import sample
def checked_empty_sample(bus,command,report,phase,**kwargs):
    state,errors=sample(bus,command,report,phase,**kwargs);bus.last_reviewed_state=state;return state,errors
class FreshEmptyBus(GuardedBus):
    checked_sample=staticmethod(checked_empty_sample)
    def approve(self,plan,proof):
        self.proof=validate_proof(plan,proof);self.plan_module=plan;self.planner=plan.validate_start;self.ring_release_actual_threshold=min(s['ring_clear_goal_ticks'] for s in self.proof['static_screens'])-4;self.ring_dip_actual_threshold=min(s['ring_clear_dip_goal_ticks'] for s in self.proof['static_screens'])-4;self.screen_approved=True
    def write(self,addr,size,values,verify=True):
        if addr==116 and getattr(self,'screen_approved',False) and getattr(self,'adopted',False) and self.report.get('phase')==ROOT_PHASE:
            state=getattr(self,'last_reviewed_state',None)
            if state is None or int(state['position'][14])>self.ring_release_actual_threshold or int(state['position'][12])>self.ring_dip_actual_threshold:raise RuntimeError('ID13/15 actual angles have not cleared the reviewed ring release; no root target advance')
        return super().write(addr,size,values,verify=verify)
FreshEmptyGraspPlan.CAMERA_TYPE=RecoveringEmptyCamera
def require_review():return validate_proof(FreshEmptyGraspPlan(REQUEST),PROOF)
if __name__=='__main__':
    if '--execute' in sys.argv:
        try:require_review()
        except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:
            print('EMPTY POSTURE NOT READY:',str(exc),'. No devices opened.',flush=True);raise SystemExit(2)
    print('EMPTY throughout. No PLACE step, cube rotation or live SAC.',flush=True)
    print('Show the WHOLE hand. One SPACE in TERMINAL starts checks/countdown; Q stops.',flush=True)
    print('Revision7: small thumb clearance first; release BOTH ring joints; slower roots. Native limits/voltage stops unchanged.',flush=True)
    print('About140s motion; original450mA/PID; goals <=2ticks/75ms. OFF/settings restored at end.',flush=True)
    raise SystemExit(main(FreshEmptyGraspPlan,bus_type=FreshEmptyBus,screener=run_screen))
