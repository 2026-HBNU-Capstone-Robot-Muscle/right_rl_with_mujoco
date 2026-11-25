"""Ordinary four-device Sync READ; retain complete-state and200ms stop policy."""
import time
import numpy as np

POLICY=dict(version='standard_sync_read4_full_health_v1_20261116',instruction=0x82,
    group_size=4,packet_window_ms=40.,sample_budget_ms=200,
    full16_required=True,each_status_error_checked=True,SDK_CRC_and_stuffing_preserved=True,
    partial_snapshot_reused=False,goal_writes=0)

def block_once(bus,address,length,fields):
    if (address,length)!=(64,83):raise ValueError('Standard reader only supports full health scope')
    for _,at,size in fields:
        if not address<=at or at+size>address+length:raise ValueError('Field outside health range')
    began=time.perf_counter();deadline=min(began+.2,getattr(bus,'_horizontal_sample_deadline',float('inf')))
    values={key:np.empty(16,dtype=np.int64) for key,_,_ in fields};completed=[]
    for first in (1,5,9,13):
        ids=list(range(first,first+4));detail=dict(completed_ids=completed.copy(),requested_ids=ids,
            health_reply_group_size=4,fast_sync_read=False,standard_sync_read=True)
        try:
            getattr(bus,'cancel_callback',lambda:None)()
            if time.perf_counter()>=deadline:raise RuntimeError('Shared200ms expired before Sync READ')
            comm=bus.packet.syncReadTx(bus.port,address,length,ids,len(ids),False)
            if comm!=bus.sdk.COMM_SUCCESS:return None,comm,0,detail
            window=min(.040,deadline-time.perf_counter())
            if window<=0:raise RuntimeError('Shared200ms expired after Sync READ transmit')
            bus.port.setPacketTimeoutMillis(window*1000)
            for motor in ids:
                data,comm,error=bus.packet.readRx(bus.port,motor,length)
                if comm!=bus.sdk.COMM_SUCCESS or error:
                    detail.update(first_incomplete_id=motor,device_error_id=motor if error else None,
                        receive_window_ms=window*1000,received_data_bytes=len(data or []),expected_data_bytes=length,
                        completed_ids=completed.copy())
                    return None,comm,error,detail
                if len(data)!=length:
                    detail.update(first_incomplete_id=motor,received_data_bytes=len(data),expected_data_bytes=length)
                    return None,bus.sdk.COMM_RX_CORRUPT,0,detail
                if time.perf_counter()>=deadline:raise RuntimeError('Complete Sync READ exceeded shared200ms')
                for key,at,size in fields:
                    values[key][motor-1]=int.from_bytes(bytes(data[at-address:at-address+size]),'little')
                completed.append(motor)
        finally:bus.port.is_using=False
    if completed!=list(range(1,17)):raise RuntimeError('Incomplete Sync READ snapshot')
    bus.report['active_health_read_policy']=dict(POLICY)
    bus.report.setdefault('standard_health_complete_reads',[]).append(dict(elapsed_ms=1000*(time.perf_counter()-began),complete_ids=completed))
    return values,bus.sdk.COMM_SUCCESS,0,{}
