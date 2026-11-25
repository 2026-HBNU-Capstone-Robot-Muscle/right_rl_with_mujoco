"""User-started recorded quarter-turn experiment. Default opens no devices."""
import sys,numpy as np
from horizontal_quarter_plan_20261027 import HorizontalQuarterPlan,REQUEST,PROOF,YAW_HOLD,RING_CLEAR,ROOTS
from loaded_quarter_reentry_20261025 import cmc_native_min
from horizontal_quarter_gate_20261027 import validate_review,run_screen
from horizontal_quarter_capture_20261027 import HorizontalQuarterCapture
from leap_new_pair_grasp_20261027 import NewPairBus,PairGraspCamera
from horizontal_quarter_health_20261027 import sample as base_sample
import camera_gap_current_runtime_20261026 as runtime
from horizontal_quarter_motion_runner_20261027 import run as confirmed_run
from horizontal_receive_resync_20261027 import receive as receive_resynchronized
from horizontal_sync_receive_window_20261028 import block_once as windowed_block,SCOPES as WINDOW_SCOPES

def sample(bus,command,report,phase,**kwargs):
    state,errors=base_sample(bus,command,report,phase,**kwargs)
    if phase==YAW_HOLD and hasattr(bus,'grasp_protocol'):bus.grasp_protocol.rotation_frame(bus,command,report,state)
    return state,errors

class HorizontalQuarterBus(NewPairBus):
    checked_sample=staticmethod(sample)
    def _block_once(self,address,length,fields):
        if (address,length) in WINDOW_SCOPES:return windowed_block(self,address,length,fields)
        return super()._block_once(address,length,fields)
    def _receive(self,operation,label,request):
        if request.get('kind')=='sync_read' and request.get('address')==64 and request.get('length')==83:
            return receive_resynchronized(self,operation,label,request)
        return super()._receive(operation,label,request)
    def approve(self,plan,proof):
        self.proof=validate_review(plan,proof);self.plan_module=plan;self.planner=plan.validate_start;self.screen_approved=True;self.support_checked=False;self.root_release_threshold=np.max(np.array([row['root_clear_goal_ticks'] for row in self.proof['empty_recovery']['static_cases']]),axis=0)+4
        self.initial_pair_checked=not self.proof['initial_self_separation']['enabled']
    def write(self,addr,size,values,verify=True):
        if addr==116 and getattr(self,'screen_approved',False) and getattr(self,'adopted',False):
            phase=self.report.get('phase');state=getattr(self,'last_reviewed_state',None)
            if phase==RING_CLEAR and (state is None or state['position'][3]<cmc_native_min()+4):raise RuntimeError('Thumb CMC not in native range before ring separation; no target advance')
            if phase==ROOTS and not getattr(self,'initial_pair_checked',True):
                threshold=self.proof['initial_self_separation']['required_actual_ring_rotation_ticks']
                if state is None or state['position'][13]<threshold:raise RuntimeError('Measured ring side clearance not reached; no root advance')
                self.initial_pair_checked=True
        try:return super().write(addr,size,values,verify=verify)
        except Exception as exc:
            if self.report.get('all_torque_off_verified') and self.report.get('primary_motion_error'):
                self.report['post_off_restore_error']=str(exc)
                raise RuntimeError(self.report['primary_motion_error']+'; torque OFF verified; settings restoration incomplete: '+str(exc)) from exc
            raise

HorizontalQuarterPlan.CAPTURE_PROTOCOL_TYPE=HorizontalQuarterCapture
HorizontalQuarterPlan.CAMERA_TYPE=PairGraspCamera

def operator_main():
    previous=runtime.run
    try:
        runtime.run=confirmed_run
        return runtime.main(HorizontalQuarterPlan,bus_type=HorizontalQuarterBus,screener=run_screen)
    finally:runtime.run=previous

if __name__=='__main__':
    try:review=validate_review(HorizontalQuarterPlan(REQUEST),PROOF)
    except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:print('NOT READY:',exc,'. No devices opened.',flush=True);raise SystemExit(2)
    print('EMPTY at FIRST SPACE. PLACE only at prompt: GREEN up, ORANGE toward WRIST/ID41.',flush=True)
    print('SECOND SPACE: grasp, recorded large horizontal turn, hold, automatic OPEN. REMOVE before THIRD SPACE = OFF.',flush=True)
    print('Experimental 90deg target; modeled responses vary. Actual cube rotation needs saved-video review. No live SAC.',flush=True)
    print('450mA/original PID and health stops unchanged. 300s active budget for the longer recorded route; Q/Esc/Ctrl+C stops.',flush=True)
    print('v10: READ recovery shares one200ms deadline; an interrupted confirmation resets the count. Two consecutive clean healthy snapshots required; no goal resend.',flush=True)
    raise SystemExit(operator_main())
