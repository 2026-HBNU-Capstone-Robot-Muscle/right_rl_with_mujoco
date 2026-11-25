"""EMPTY-only sampler: confirm recovered voltage without an extra nap.

The150ms budget, six-read limit, two >=3.8V confirmations and every existing
motor-health criterion remain unchanged. No motor writes are performed here.
"""
import time
import numpy as np
import sequence_flip_checked as checked

def metadata():
    return dict(version='EMPTY_immediate_second_healthy_voltage_read_v1_20261026',
        empty_only=True,confirmation_budget_s=.15,maximum_confirmation_reads=6,
        recover_tenths=38,consecutive_healthy_reads=2,
        nonrecovered_poll_delay_s=.01,first_healthy_followup_delay_s=0.,
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
        from goal_readback_confirmation import confirm_previous_goal
        return confirm_previous_goal(bus,command,state,report,phase,
            read_sample=lambda:checked._read_sample(bus,command,report,phase,snapshot_check),
            validate=lambda fresh:checked.health(fresh,command,**profile))
    except checked.MildUndervoltage:pass
    began=time.monotonic()
    event=dict(phase=phase,t=began-report['enabled_at'],status='confirming',
        initial_voltage_v=(state['voltage']/10).tolist(),confirmation_reads=0,goals_advanced=0,
        policy=metadata(),healthy_confirmation_count=0)
    report['fresh_voltage_read_policy']=metadata()
    events=report.setdefault('voltage_events',[]);events.append(event)
    try:
        if sum(event['t']-e['t']<=checked.VOLTAGE_REPEAT_WINDOW_S for e in events)>=checked.VOLTAGE_REPEAT_TRIP:
            raise RuntimeError('Repeated undervoltage: 3 events within 2s; torque OFF')
        print('Voltage dip: hold last goal;150ms budget. First healthy voltage is re-read immediately.',flush=True)
        good=0
        for _ in range(checked.VOLTAGE_CONFIRM_MAX_READS):
            getattr(bus,'cancel_callback',checked.check_cancel)()
            if time.monotonic()-began>=checked.VOLTAGE_CONFIRM_S:break
            state,errors=checked._read_sample(bus,command,report,phase,snapshot_check)
            event['confirmation_reads']+=1;event['last_voltage_v']=(state['voltage']/10).tolist()
            try:checked.health(state,command,**profile)
            except checked.MildUndervoltage:good=0
            else:good=good+1 if np.all(state['voltage']>=checked.VOLTAGE_RECOVER_TENTHS) else 0
            elapsed=time.monotonic()-began
            event['healthy_confirmation_count']=good
            if elapsed>=checked.VOLTAGE_CONFIRM_S:break
            if good>=checked.VOLTAGE_RECOVER_READS:
                event.update(status='recovered',elapsed_ms=elapsed*1000)
                print('Voltage recovered: all IDs >=3.8V twice within150ms; continuing from same step.',flush=True)
                return state,errors
            # A second fresh normal reading does not require a software delay.
            # Continue the old10ms pacing only while voltage has not recovered.
            if good==0:time.sleep(.01)
        raise RuntimeError('Undervoltage not recovered to >=3.8V twice within150ms/read limit; torque OFF')
    except (Exception,KeyboardInterrupt) as exc:
        event.update(status='stopped',elapsed_ms=(time.monotonic()-began)*1000,error=str(exc) or 'Interrupted')
        raise

def sample(bus,command,report,phase,*,snapshot_check=None):
    plan=getattr(bus,'plan_module',None)
    if not getattr(plan,'IMMEDIATE_HEALTHY_VOLTAGE_RECHECK',False) or getattr(plan,'CAPTURE_PLACEMENT',False):
        raise RuntimeError('Immediate healthy voltage confirmation is EMPTY-only; no target advance')
    began=time.monotonic();failures=len(report.get('read_failures',[]))
    state,errors=_healthy(bus,command,report,phase,snapshot_check=snapshot_check)
    if (getattr(plan,'CONFIRM_FRESH_AFTER_READ_RECOVERY',False)
        and len(report.get('read_failures',[]))>failures):
        from read_recovery_confirmation import confirm_recovered_read
        profile=getattr(bus,'checked_control_profile',{})
        return confirm_recovered_read(bus,command,state,errors,report,phase,began=began,
            read_sample=lambda:checked._read_sample(bus,command,report,phase,snapshot_check),
            validate=lambda fresh:checked.health(fresh,command,**profile),now=time.monotonic)
    return state,errors
