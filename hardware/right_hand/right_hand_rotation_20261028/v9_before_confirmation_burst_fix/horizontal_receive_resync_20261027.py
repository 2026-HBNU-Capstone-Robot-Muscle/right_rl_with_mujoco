"""Bounded READ-only resynchronization; never resend a motion/setting write."""
import time
POLICY=dict(version='full_snapshot_quiet_resync_v1_20261027',maximum_requests=3,
    total_read_budget_s=.2,minimum_retry_reserve_s=.1,quiet_s=.003,maximum_drain_s=.02,
    maximum_retries_per_10s=3,scope_address=64,scope_length=83,
    consecutive_healthy_snapshots_after_recovery=2,goal_or_setting_write_retries=0)

def quiet(bus,*,now=time.monotonic,sleep=time.sleep):
    port=bus.port;began=last_data=now();discarded=0
    cancel=getattr(bus,'cancel_callback',lambda:None)
    while True:
        cancel();stamp=now()
        if stamp-began>=.02:raise RuntimeError('Serial input did not become quiet; no target advance')
        if getattr(port,'is_using',False):raise RuntimeError('Serial port busy during resynchronization')
        count=port.getBytesAvailable()
        if count:
            discarded+=len(port.readPort(count));last_data=now()
        elif stamp-last_data>=.003:
            return dict(discarded_bytes=discarded,elapsed_ms=(stamp-began)*1000)
        sleep(.001)

def receive(bus,operation,label,request,*,now=time.monotonic,sleep=time.sleep):
    if request.get('kind')!='sync_read' or request.get('address')!=64 or request.get('length')!=83:
        raise ValueError('Resynchronization is limited to full health READs')
    began=now();failures=[]
    for attempt in range(3):
        getattr(bus,'cancel_callback',lambda:None)()
        value,comm,error,detail=operation();stamp=now()
        if comm==bus.sdk.COMM_SUCCESS and not error:
            if stamp-began>.2:raise RuntimeError(f'{label}: read exceeded200ms; no target advance')
            for event in failures:event.update(recovered=True,total_elapsed_ms=(stamp-began)*1000)
            return value
        event=dict(request,**detail,attempt=attempt+1,comm=int(comm),device_error=int(error),
            elapsed_ms=(stamp-began)*1000,retry_requested=False,recovered=False,
            recovery_policy=POLICY['version'])
        bus.report.setdefault('read_failures',[]).append(event);failures.append(event)
        bus._read_recovery_times=[t for t in bus._read_recovery_times if stamp-t<10.]
        retry=(comm in (-3001,-3002) and not error and attempt<2
            and stamp-began+.02+.1<.2 and len(bus._read_recovery_times)<3)
        if not retry:
            raise RuntimeError(f'{label}: comm={comm}, error={error}; no target advance')
        event['quiet_resynchronization']=quiet(bus,now=now,sleep=sleep)
        bus._read_recovery_times.append(now());event['retry_requested']=True
    raise RuntimeError('Bounded serial recovery exhausted; no target advance')
