"""Allow actual quiet/drain time before requiring the existing100ms reserve.

The200ms total deadline, three READ requests, retry rate, full fresh snapshots,
and two healthy confirmations remain unchanged. No motion writes are retried.
"""
import time
from calibrated_precise_receive_20261028 import quiet

POLICY=dict(version='quiet_then_existing100ms_reserve_QPC_v1_20261029',
    budget_clock='time.perf_counter',maximum_requests=3,total_read_budget_s=.2,
    minimum_retry_reserve_s=.1,quiet_s=.003,maximum_drain_s=.02,
    maximum_retries_per10s=3,scope_address=64,scope_length=83,
    consecutive_healthy_snapshots_after_recovery=2,goal_or_setting_write_retries=0,
    missing_or_partial_snapshot_accepted=False,shared_sample_deadline_respected=True,
    actual_quiet_time_charged_before_reserve_check=True)

def receive(bus,operation,label,request,*,now=time.perf_counter,sleep=time.sleep):
    if request.get('kind')!='sync_read' or request.get('address')!=64 or request.get('length')!=83:
        raise ValueError('Recovery limited to complete health READs')
    began=now();deadline=min(began+.2,getattr(bus,'_horizontal_sample_deadline',began+.2));failures=[]
    for attempt in range(3):
        getattr(bus,'cancel_callback',lambda:None)()
        if now()>=deadline:raise RuntimeError(label+': read exceeded200ms shared deadline; no target advance')
        value,comm,error,detail=operation();stamp=now()
        if comm==bus.sdk.COMM_SUCCESS and not error:
            if stamp>=deadline:raise RuntimeError(label+': read exceeded200ms shared deadline; no target advance')
            for event in failures:event.update(recovered=True,total_elapsed_ms=(stamp-began)*1000)
            return value
        event=dict(request,**detail,attempt=attempt+1,comm=int(comm),device_error=int(error),
            elapsed_ms=(stamp-began)*1000,retry_requested=False,recovered=False,recovery_policy=POLICY['version'])
        bus.report.setdefault('read_failures',[]).append(event);failures.append(event)
        bus._read_recovery_times=[t for t in bus._read_recovery_times if stamp-t<10.]
        allowed=(comm in (-3001,-3002) and not error and attempt<2 and len(bus._read_recovery_times)<3)
        # A quiet line can cost3ms, not necessarily its20ms upper bound. Still
        # require the original100ms AFTER the actual bounded drain has finished.
        if not allowed or stamp+.003+.1>=deadline:
            raise RuntimeError(f'{label}: comm={comm}, error={error}; no target advance')
        event['quiet_resynchronization']=quiet(bus,now=now,sleep=sleep)
        event['remaining_budget_after_quiet_ms']=(deadline-now())*1000
        if now()+.1>=deadline:
            event['reserve_check']='insufficient_after_actual_quiet'
            raise RuntimeError(label+': existing100ms read reserve unavailable after quiet; no target advance')
        event['reserve_check']='existing100ms_available_after_actual_quiet'
        bus._read_recovery_times.append(now());event['retry_requested']=True
    raise RuntimeError('Bounded READ recovery exhausted; no target advance')
