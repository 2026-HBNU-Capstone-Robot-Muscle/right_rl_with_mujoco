"""Operator-started new grasp observation; default opens no device."""
import sys,numpy as np
from new_pair_grasp_plan_20261027 import NewPairGraspPlan,REQUEST,PROOF,ROOTS,JOIN,OPEN,HOLD
from new_pair_grasp_gate_20261027 import validate_proof,run_screen
from loaded_quarter_reentry_20261025 import cmc_native_min
from loaded_repeat_voltage_20261026 import _healthy
from read_recovery_confirmation import confirm_recovered_read
from leap_thumb_prepare_current_empty import Bus as GuardedBus
from camera_gap_current_runtime_20261026 import main
from new_pair_grasp_capture_20261027 import NewPairGraspCapture
from quarter_finish_empty_camera_20261025 import QuarterFinishEmptyCamera
import sequence_flip_checked as checked,time
def sample(bus,command,report,phase,**kwargs):
    if not getattr(bus.plan_module,'NEW_PAIR_GRASP_MODE',False):raise RuntimeError('Wrong grasp mode; no target advance')
    began=time.monotonic();failures=len(report.get('read_failures',[]));state,errors=_healthy(bus,command,report,phase,**kwargs)
    if len(report.get('read_failures',[]))>failures:state,errors=confirm_recovered_read(bus,command,state,errors,report,phase,began=began,read_sample=lambda:checked._read_sample(bus,command,report,phase,kwargs.get('snapshot_check')),validate=lambda fresh:checked.health(fresh,command,**getattr(bus,'checked_control_profile',{})),now=time.monotonic)
    bus.last_reviewed_state=state
    if phase==HOLD and hasattr(bus,'grasp_protocol'):bus.grasp_protocol.hold_frame(bus,command,report,state)
    return state,errors
class NewPairBus(GuardedBus):
    checked_sample=staticmethod(sample)
    def approve(self,plan,proof):
        self.proof=validate_proof(plan,proof);self.plan_module=plan;self.planner=plan.validate_start;self.screen_approved=True;self.support_checked=False;self.root_release_threshold=np.max(np.array([s['root_clear_goal_ticks'] for s in self.proof['static_screens']]),axis=0)+4
    def write(self,addr,size,values,verify=True):
        if addr==116 and getattr(self,'screen_approved',False) and getattr(self,'adopted',False):
            phase=self.report.get('phase');state=getattr(self,'last_reviewed_state',None)
            if phase==ROOTS and (state is None or state['position'][3]<cmc_native_min()+4):raise RuntimeError('ID4 has not returned inside its configured range; no root advance')
            if phase in (JOIN,OPEN) and not self.support_checked:
                if state is None or np.any(state['position'][[7,11,15]]<self.root_release_threshold):raise RuntimeError('Actual roots have not cleared the reviewed support; no finger opening advance')
                self.support_checked=True
        return super().write(addr,size,values,verify=verify)
class PairGraspCamera(QuarterFinishEmptyCamera):
    EMPTY_ONLY=False
NewPairGraspPlan.CAPTURE_PROTOCOL_TYPE=NewPairGraspCapture
NewPairGraspPlan.CAMERA_TYPE=PairGraspCamera
if __name__=='__main__':
    if '--execute' in sys.argv:
        try:validate_proof(NewPairGraspPlan(REQUEST),PROOF)
        except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:print('NOT READY:',exc,'. No devices opened.',flush=True);raise SystemExit(2)
    print('THREE SPACEs: EMPTY start; PLACE cube/hands clear; REMOVE after automatic OPEN. No rotation/SAC.',flush=True)
    print('Current OFF angles read automatically. Existing passive thumb reentry; original450mA/PID and health stops.',flush=True)
    raise SystemExit(main(NewPairGraspPlan,bus_type=NewPairBus,screener=run_screen))
