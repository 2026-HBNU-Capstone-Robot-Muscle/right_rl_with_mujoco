"""READ resynchronization uses the existing shared200ms health deadline.

Independent20ms drain cutoffs and predicted100ms reservations are removed.
No new goal can advance until two complete healthy snapshots fit the SAME200ms.
The request/rate limits, driver reply windows and all motor health stops remain.
"""
import time

POLICY=dict(version='shared200ms_quiet_and_two_complete_QPC_v2_20261029',
    budget_clock='time.perf_counter',maximum_requests=3,total_read_budget_s=.2,
    quiet_s=.003,quiet_uses_remaining_shared_sample_budget=True,
    independent_drain_deadline_s=None,predicted_retry_reservation_s=None,
    maximum_retries_per10s=3,scope_address=64,scope_length=83,
    consecutive_healthy_snapshots_after_recovery=2,goal_or_setting_write_retries=0,
    missing_or_partial_snapshot_accepted=False,shared_sample_deadline_respected=True,
    resynchronization_trace_recorded=True)

def quiet(bus,deadline,trace,*,now=time.perf_counter,sleep=time.sleep):
    began=last_data=now();discarded=0;observations=[]
    trace.update(status='waiting',discarded_bytes=0,observations=observations)
    try:
        while True:
            getattr(bus,'cancel_callback',lambda:None)();stamp=now()
            if stamp>=deadline:raise RuntimeError('Shared200ms expired during READ resynchronization; no target advance')
            if getattr(bus.port,'is_using',False):raise RuntimeError('Serial port busy during resynchronization')
            count=bus.port.getBytesAvailable()
            if count:
                discarded+=len(bus.port.readPort(count));last_data=now()
            observations.append(dict(elapsed_ms=(stamp-began)*1000,available_bytes=int(count)))
            trace.update(elapsed_ms=(now()-began)*1000,discarded_bytes=discarded)
            if not count and stamp-last_data>=.003:
                trace.update(status='quiet',remaining_shared_budget_ms=(deadline-now())*1000);return trace
            sleep(min(.001,max(0.,deadline-now())))
    except (Exception,KeyboardInterrupt) as exc:
        trace.update(status='stopped',elapsed_ms=(now()-began)*1000,error=str(exc));raise

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
        if not allowed or stamp>=deadline:
            raise RuntimeError(f'{label}: comm={comm}, error={error}; no target advance')
        event['quiet_resynchronization']={}
        quiet(bus,deadline,event['quiet_resynchronization'],now=now,sleep=sleep)
        event['remaining_budget_after_quiet_ms']=(deadline-now())*1000
        if now()>=deadline:raise RuntimeError(label+': shared200ms expired before full READ retry; no target advance')
        bus._read_recovery_times.append(now());event['retry_requested']=True
    raise RuntimeError('Bounded READ recovery exhausted; no target advance')
