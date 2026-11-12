"""Two complete healthy snapshots after the last bad read, within200ms.

The recovered snapshot is produced by an entirely new16-motor request after
quiet resynchronization. Its partial predecessor is never reused. It counts
as the first healthy snapshot only after every original health/goal check.
"""
import time
import numpy as np
from horizontal_read_recovery_20261028 import sample_deadline
from horizontal_quarter_health_20261027 import _healthy
import sequence_flip_checked as checked
from new_pair_grasp_plan_20261027 import HOLD
from horizontal_quarter_plan_20261027 import YAW_HOLD
POLICY=dict(version='two_complete_after_last_bad_read_QPC_v1_20261028',sample_budget_s=.2,
    required_consecutive_clean_reads=2,maximum_confirmation_reads=5,
    recovered_complete_snapshot_counts_as_first_healthy=True,partial_snapshot_counts_as_healthy=False,
    new_recovered_failure_resets_to_one_after_fresh_full_validation=True,
    existing_transport_retry_rate_limit_preserved=True,goal_or_setting_writes=0,budget_clock='time.perf_counter')
def confirm(bus,command,state,errors,report,phase,*,began,read_sample,validate,now=time.perf_counter):
    command=np.asarray(command).copy()
    event=dict(version=POLICY['version'],policy=dict(POLICY),phase=phase,status='confirming',command=command.tolist(),
        goals_advanced=0,goal_writes=0,clean_reads=0,required_clean_reads=2,sample_budget_ms=200,
        sample_indices=[],consecutive_clean_sample_indices=[],confirmation_reads=0,clean_count_resets=0)
    report.setdefault('read_recovery_confirmation_events',[]).append(event)
    def check(fresh):
        validate(fresh)
        if not np.array_equal(fresh['goal'],command):raise RuntimeError('Goal changed during read recovery confirmation; OFF')
        if now()>=min(began+.2,getattr(bus,'_horizontal_sample_deadline',began+.2)):raise RuntimeError('Recovered confirmation exceeded200ms; OFF')
    def index(role):
        if report.get('samples'):
            i=len(report['samples'])-1;report['samples'][i]['sample_role']=role;event['sample_indices'].append(i);return i
        return None
    try:
        check(state);first=index('recovered_complete_first_confirmation')
        event.update(clean_reads=1,consecutive_clean_sample_indices=[first])
        for _ in range(POLICY['maximum_confirmation_reads']):
            getattr(bus,'cancel_callback',lambda:None)();check(state)
            before=len(report.get('read_failures',[]));state,errors=read_sample();event['confirmation_reads']+=1;check(state)
            new=report.get('read_failures',[])[before:]
            if new and not all(v.get('recovered') is True for v in new):raise RuntimeError('Unrecovered communication failure; OFF')
            i=index('post_read_recovery_confirmation')
            if new:
                event['clean_reads']=1;event['consecutive_clean_sample_indices']=[i];event['clean_count_resets']+=1
            else:
                event['clean_reads']+=1;event['consecutive_clean_sample_indices'].append(i)
            if event['clean_reads']>=2:
                event.update(status='confirmed',elapsed_ms=(now()-began)*1000);return state,errors
        raise RuntimeError('Two complete healthy reads not confirmed within read limit; OFF')
    except (Exception,KeyboardInterrupt) as exc:
        event.update(status='stopped',elapsed_ms=(now()-began)*1000,error=str(exc) or 'Interrupted')
        if isinstance(exc,KeyboardInterrupt):raise
        raise RuntimeError('Fresh-state confirmation failed; no target advance: '+str(exc)) from exc
def sample(bus,command,report,phase,**kwargs):
    if not getattr(bus.plan_module,'NEW_PAIR_GRASP_MODE',False):raise RuntimeError('Wrong grasp mode; no target advance')
    began=time.perf_counter();failures=len(report.get('read_failures',[]))
    with sample_deadline(bus,began):
        state,errors=_healthy(bus,command,report,phase,**kwargs)
        if len(report.get('read_failures',[]))>failures:
            state,errors=confirm(bus,command,state,errors,report,phase,began=began,
                read_sample=lambda:checked._read_sample(bus,command,report,phase,kwargs.get('snapshot_check')),
                validate=lambda fresh:checked.health(fresh,command,**getattr(bus,'checked_control_profile',{})),now=time.perf_counter)
    bus.last_reviewed_state=state
    if hasattr(bus,'grasp_protocol'):
        if phase==HOLD:bus.grasp_protocol.hold_frame(bus,command,report,state)
        if phase==YAW_HOLD:bus.grasp_protocol.rotation_frame(bus,command,report,state)
    return state,errors
