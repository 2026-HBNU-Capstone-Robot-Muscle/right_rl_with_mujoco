"""Bound reply bursts while retaining complete16-motor health snapshots."""
import time
import numpy as np
SCOPES=((64,83),(64,1))
POLICY=dict(version='bounded_sync_receive_window_v4_single_motor_replies_20261028',packet_window_ms=80.,
    scopes=[list(v) for v in SCOPES],overall_health_sample_budget_ms=200,
    two_fresh_healthy_snapshots_after_recovery=True,goal_or_setting_write_retries=0,
    missing_or_corrupt_data_accepted=False,hardware_health_limits_unchanged=True,
    per_packet_window_capped_by_sample_deadline=True,full_health_group_size=1,
    torque_off_readback_group_size=16,all16_fresh_before_return=True,
    reused_half_snapshots_after_failure=False)

def block_once(bus,address,length,fields):
    if (address,length) not in SCOPES:raise ValueError('Outside bounded receive-window scope')
    began=time.perf_counter();bus.report['sync_receive_window_policy']=dict(POLICY)
    deadline=min(time.monotonic()+.2,getattr(bus,'_horizontal_sample_deadline',float('inf')))
    count=1 if length==83 else 16
    values={name:np.empty(16,dtype=np.int64) for name,_,_ in fields};completed=[]
    for start in range(1,17,count):
        ids=list(range(start,min(start+count,17)))
        group=bus.sdk.GroupSyncRead(bus.port,bus.packet,address,length);window_ms=80.
        try:
            if time.monotonic()>=deadline:raise RuntimeError('Health sample deadline exhausted before READ; no target advance')
            for motor in ids:
                if not group.addParam(motor):raise RuntimeError(f'Cannot add ID{motor} to health READ')
            comm=group.txPacket()
            if comm==bus.sdk.COMM_SUCCESS:
                window_ms=min(80.,1000*(deadline-time.monotonic()))
                if window_ms<=0:raise RuntimeError('Health sample deadline exhausted after READ transmission; no target advance')
                bus.port.setPacketTimeoutMillis(window_ms);comm=group.rxPacket()
            if comm!=bus.sdk.COMM_SUCCESS:
                partial=[i for i,data in getattr(group,'data_dict',{}).items() if len(data)==length]
                received=completed+partial
                return None,comm,0,dict(completed_ids=received,
                    first_incomplete_id=next((i for i in range(1,17) if i not in received),None),
                    receive_window_ms=window_ms,requested_ids=ids,health_reply_group_size=count)
            for motor in ids:
                for name,addr,size in fields:
                    if not group.isAvailable(motor,addr,size):raise RuntimeError(f'ID{motor} data unavailable at{addr}')
                    values[name][motor-1]=int(group.getData(motor,addr,size))
                completed.append(motor)
        finally:group.clearParam()
    if time.monotonic()>=deadline:raise RuntimeError('Complete health snapshot exceeded shared200ms deadline; no target advance')
    if completed!=list(range(1,17)):raise RuntimeError('Incomplete combined health snapshot; no target advance')
    elapsed=(time.perf_counter()-began)*1000
    if elapsed>40:
        bus.report.setdefault('slow_complete_sync_reads',[]).append(dict(address=address,length=length,
            elapsed_ms=elapsed,complete_ids=completed,receive_window_ms=80.,health_reply_group_size=count))
    return values,comm,0,{}
