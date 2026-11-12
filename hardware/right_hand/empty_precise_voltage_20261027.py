"""EMPTY-only use of the unchanged precise voltage/readback confirmation."""
import time
import sequence_flip_checked as checked
from loaded_repeat_voltage_20261026 import _healthy,metadata as source_metadata
def metadata():
    return dict(version='EMPTY_precise_read_paced_voltage_v1_20261027',empty_only=True,source_policy=source_metadata(),thresholds_and_read_budget_changed=False,goal_resends=0)
def sample(bus,command,report,phase,*,snapshot_check=None):
    plan=getattr(bus,'plan_module',None)
    if not getattr(plan,'FRESH_EMPTY_GRASP_MODE',False) or getattr(plan,'CAPTURE_PLACEMENT',False):raise RuntimeError('This sampler requires the reviewed EMPTY grasp mode')
    began=time.monotonic();failures=len(report.get('read_failures',[]));report['empty_voltage_policy']=metadata();state,errors=_healthy(bus,command,report,phase,snapshot_check=snapshot_check)
    if len(report.get('read_failures',[]))>failures:
        from read_recovery_confirmation import confirm_recovered_read
        profile=getattr(bus,'checked_control_profile',{})
        return confirm_recovered_read(bus,command,state,errors,report,phase,began=began,read_sample=lambda:checked._read_sample(bus,command,report,phase,snapshot_check),validate=lambda fresh:checked.health(fresh,command,**profile),now=time.monotonic)
    return state,errors
