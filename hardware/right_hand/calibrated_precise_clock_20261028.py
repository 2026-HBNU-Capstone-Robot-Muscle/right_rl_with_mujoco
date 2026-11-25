"""Per-instance high-resolution host clocks. No motor/setting writes."""
import time
POLICY=dict(version='calibrated_QPC_clock_v1_20261028',budget_clock='time.perf_counter',
    driver_timeout_clock='time.perf_counter milliseconds',legacy_log_clock_unchanged=True,
    sample_budget_ms=200,wall_clock_or_GetTickCount_not_used_for_health_budget=True)
def install(bus):
    if getattr(bus.port,'is_using',False):raise RuntimeError('Cannot change host clock during a transaction')
    bus.port.getCurrentTime=lambda:time.perf_counter()*1000.
    bus.report['precise_transport_clock_policy']=dict(POLICY)
def receive_other(bus,operation,label,request):
    """Original scalar-read recovery semantics, using the same QPC retry epoch."""
    began=time.perf_counter();recovery=None
    for attempt in range(2):
        value,comm,error,detail=operation();stamp=time.perf_counter()
        if comm==bus.sdk.COMM_SUCCESS and not error:
            if bus.read_timeout_recovery and stamp-began>.2:raise RuntimeError(f'{label}: read exceeded200ms; no target advance')
            if recovery is not None:recovery.update(recovered=True,total_elapsed_ms=(stamp-began)*1000)
            return value
        event=dict(request,**detail,attempt=attempt+1,comm=comm,device_error=error,elapsed_ms=(stamp-began)*1000,retry_requested=False,recovered=False)
        bus.report.setdefault('read_failures',[]).append(event)
        bus._read_recovery_times=[t for t in bus._read_recovery_times if stamp-t<10.]
        retry=(bus.read_timeout_recovery and (comm==-3001 or (bus.read_corrupt_recovery and comm==-3002)) and not error and attempt==0 and stamp-began<.1 and len(bus._read_recovery_times)<3)
        if not retry:raise RuntimeError(f'{label}: comm={comm}, error={error}; no target advance')
        bus._read_recovery_times.append(stamp);event['retry_requested']=True;recovery=event
    raise RuntimeError('Read recovery exhausted; no target advance')
def review():
    import hashlib,json
    from pathlib import Path
    root=Path(__file__).resolve().parent
    path=root/'precise_two_full_readonly_20261028_060908_257593/result.json'
    r=json.loads(path.read_bytes())
    if r.get('status')!='precise_two_full_readonly_complete' or r.get('reads')!=200 or r.get('motor_writes')!=0 or r.get('torque_changes')!=0 or r.get('read_failures',[]):raise RuntimeError('Precise transport READ-only evidence incomplete')
    if r.get('precise_transport_clock_policy')!=POLICY or r.get('frames',0)<100 or r.get('bad_frames')!=0 or len(r.get('latencies_ms',[]))!=200 or max(r['latencies_ms'])>=200:raise RuntimeError('Precise clock/camera/read budget evidence rejected')
    return dict(source=path.relative_to(root).as_posix(),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        snapshots=200,read_failures=0,camera_frames=r['frames'],maximum_sample_ms=max(r['latencies_ms']),
        installed_policy=dict(POLICY),motor_writes=0,physical_motion_recovery_verified=False)
