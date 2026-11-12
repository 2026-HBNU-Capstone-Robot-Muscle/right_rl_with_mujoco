"""Allow delayed complete responses inside the existing200ms health budget.

Only full health READs and torque-OFF readback use this host receive window.
No motor settings, packet format, requested fields or health thresholds change.
"""
import time
import numpy as np
SCOPES=((64,83),(64,1))
POLICY=dict(version='bounded_sync_receive_window_v1_20261028',packet_window_ms=80.,
    scopes=[list(v) for v in SCOPES],overall_health_sample_budget_ms=200,
    two_fresh_healthy_snapshots_after_recovery=True,goal_or_setting_write_retries=0,
    missing_or_corrupt_data_accepted=False,hardware_health_limits_unchanged=True)

def block_once(bus,address,length,fields):
    if (address,length) not in SCOPES:raise ValueError('Outside bounded receive-window scope')
    group=bus.sdk.GroupSyncRead(bus.port,bus.packet,address,length)
    began=time.perf_counter();bus.report['sync_receive_window_policy']=dict(POLICY)
    try:
        for motor in range(1,17):
            if not group.addParam(motor):raise RuntimeError(f'Cannot add ID{motor} to health READ')
        comm=group.txPacket()
        if comm==bus.sdk.COMM_SUCCESS:
            bus.port.setPacketTimeoutMillis(80.)
            comm=group.rxPacket()
        if comm!=bus.sdk.COMM_SUCCESS:
            completed=[i for i,data in getattr(group,'data_dict',{}).items() if len(data)==length]
            return None,comm,0,dict(completed_ids=completed,
                first_incomplete_id=next((i for i in range(1,17) if i not in completed),None),receive_window_ms=80.)
        values={}
        for name,addr,size in fields:
            row=[]
            for motor in range(1,17):
                if not group.isAvailable(motor,addr,size):raise RuntimeError(f'ID{motor} data unavailable at{addr}')
                row.append(int(group.getData(motor,addr,size)))
            values[name]=np.asarray(row,dtype=np.int64)
        elapsed=(time.perf_counter()-began)*1000
        if elapsed>40:
            bus.report.setdefault('slow_complete_sync_reads',[]).append(dict(address=address,length=length,
                elapsed_ms=elapsed,complete_ids=list(range(1,17)),receive_window_ms=80.))
        return values,comm,0,{}
    finally:group.clearParam()
