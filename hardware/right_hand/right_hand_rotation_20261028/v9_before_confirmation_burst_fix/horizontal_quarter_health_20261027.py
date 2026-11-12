"""Supervised loaded sampler: confirm recovered voltage without an extra nap.

The150ms budget, six-read limit, two >=3.8V confirmations and every existing
motor-health criterion remain unchanged. No motor writes are performed here.
"""
import time
import numpy as np
import sequence_flip_checked as checked

def metadata():
    return dict(version='LOADED_precise_read_paced_voltage_v1_20261026',
        empty_only=False,confirmation_budget_s=.15,maximum_confirmation_reads=6,
        recover_tenths=38,consecutive_healthy_reads=2,
        nonrecovered_minimum_poll_cycle_s=.01,read_work_included_in_poll_cycle=True,first_healthy_followup_delay_s=0.,
        confirmation_clock='time.perf_counter',clock_epoch_not_mixed_with_enabled_at=True,
        initial_3p6_only=True,immediate_stop_at_or_below_3p5_unchanged=True,
        repeated_dip_window_s=2.,repeated_dip_trip=3,
        goal_advance_or_resend_during_confirmation=False,other_health_checks_unchanged=True)

def _healthy(bus,command,report,phase,*,snapshot_check=None):
    state,errors=checked._read_sample(bus,command,report,phase,snapshot_check)
    profile=getattr(bus,'checked_control_profile',{})
    try:
        checked.health(state,command,**profile)
        return state,errors
    except checked.GoalReadbackMismatch:
        if not getattr(bus,'goal_readback_confirmation',False):raise
        from horizontal_goal_confirmation_20261027 import confirm_previous_goal
        return confirm_previous_goal(bus,command,state,report,phase,
            read_sample=lambda:checked._read_sample(bus,command,report,phase,snapshot_check),
            validate=lambda fresh:checked.health(fresh,command,**profile))
    except checked.MildUndervoltage:pass
    began=time.perf_counter()
    event=dict(phase=phase,t=time.monotonic()-report['enabled_at'],status='confirming',
        initial_voltage_v=(state['voltage']/10).tolist(),confirmation_reads=0,goals_advanced=0,
        policy=metadata(),healthy_confirmation_count=0)
    report['fresh_voltage_read_policy']=metadata()
    events=report.setdefault('voltage_events',[]);events.append(event)
    try:
        if sum(event['t']-e['t']<=checked.VOLTAGE_REPEAT_WINDOW_S for e in events)>=checked.VOLTAGE_REPEAT_TRIP:
            raise RuntimeError('Repeated undervoltage: 3 events within 2s; torque OFF')
        print('Voltage dip: hold goals;150ms precise budget. Read work counts toward the10ms polling cycle.',flush=True)
        good=0
        for _ in range(checked.VOLTAGE_CONFIRM_MAX_READS):
            getattr(bus,'cancel_callback',checked.check_cancel)()
            if time.perf_counter()-began>=checked.VOLTAGE_CONFIRM_S:break
            read_began=time.perf_counter()
            state,errors=checked._read_sample(bus,command,report,phase,snapshot_check)
            event['confirmation_reads']+=1;event['last_voltage_v']=(state['voltage']/10).tolist()
            try:checked.health(state,command,**profile)
            except checked.MildUndervoltage:good=0
            else:good=good+1 if np.all(state['voltage']>=checked.VOLTAGE_RECOVER_TENTHS) else 0
            elapsed=time.perf_counter()-began
            event['healthy_confirmation_count']=good
            if elapsed>=checked.VOLTAGE_CONFIRM_S:break
            if good>=checked.VOLTAGE_RECOVER_READS:
                event.update(status='recovered',elapsed_ms=elapsed*1000)
                print('Voltage recovered: all IDs >=3.8V twice within150ms; continuing from same step.',flush=True)
                return state,errors
            # A second fresh normal reading does not require a software delay.
            # The10ms minimum retry cycle includes the blocking full-health READ.
            if good==0:
                remaining=max(0.,.01-(time.perf_counter()-read_began))
                if remaining:time.sleep(remaining)
        raise RuntimeError('Undervoltage not recovered to >=3.8V twice within150ms/read limit; torque OFF')
    except (Exception,KeyboardInterrupt) as exc:
        event.update(status='stopped',elapsed_ms=(time.perf_counter()-began)*1000,error=str(exc) or 'Interrupted')
        raise

from read_recovery_confirmation import confirm_recovered_read
from new_pair_grasp_plan_20261027 import HOLD

def sample(bus,command,report,phase,**kwargs):
    if not getattr(bus.plan_module,'NEW_PAIR_GRASP_MODE',False):raise RuntimeError('Wrong grasp mode; no target advance')
    began=time.monotonic();failures=len(report.get('read_failures',[]));state,errors=_healthy(bus,command,report,phase,**kwargs)
    if len(report.get('read_failures',[]))>failures:state,errors=confirm_recovered_read(bus,command,state,errors,report,phase,began=began,read_sample=lambda:checked._read_sample(bus,command,report,phase,kwargs.get('snapshot_check')),validate=lambda fresh:checked.health(fresh,command,**getattr(bus,'checked_control_profile',{})),now=time.monotonic)
    bus.last_reviewed_state=state
    if phase==HOLD and hasattr(bus,'grasp_protocol'):bus.grasp_protocol.hold_frame(bus,command,report,state)
    return state,errors
