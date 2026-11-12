"""Four smaller Fast READ replies; identical strict decoder and200ms deadline."""
import time
import numpy as np
from fast_health_read_20261029 import decode_status,DeviceStatusError,POLICY as BASE_POLICY
GROUPS=tuple(tuple(range(n,n+4)) for n in (1,5,9,13))
POLICY=dict(BASE_POLICY,version='strict_fast_sync_read4_QPC_v1_20261029',group_size=4,maximum_reply_bytes=356)
def block_once(bus,address,length,fields):
    if (address,length)!=(64,83):raise ValueError('Fast read limited to full health scope')
    began=time.perf_counter();deadline=min(began+.2,getattr(bus,'_horizontal_sample_deadline',float('inf')))
    values={name:np.empty(16,dtype=np.int64) for name,_,_ in fields};completed=[]
    bus.report['fast_health_read_policy']=dict(POLICY)
    for group in GROUPS:
        ids=list(group);window=40.
        try:
            getattr(bus,'cancel_callback',lambda:None)()
            if time.perf_counter()>=deadline:raise RuntimeError('Shared200ms expired before Fast READ; no target advance')
            comm=bus.packet.syncReadTx(bus.port,address,length,ids,len(ids),True)
            if comm!=bus.sdk.COMM_SUCCESS:return None,comm,0,dict(completed_ids=completed,requested_ids=ids,health_reply_group_size=4,fast_sync_read=True)
            window=min(40.,1000*(deadline-time.perf_counter()))
            if window<=0:raise RuntimeError('Shared200ms expired after Fast READ transmit; no target advance')
            bus.port.setPacketTimeoutMillis(window);raw,comm=bus.packet.rxPacket(bus.port,True)
            if comm!=bus.sdk.COMM_SUCCESS:return None,comm,0,dict(completed_ids=completed,first_incomplete_id=ids[0],requested_ids=ids,receive_window_ms=window,
                health_reply_group_size=4,fast_sync_read=True,received_packet_bytes=len(raw or []),expected_packet_bytes=356)
            try:decoded=decode_status(bus.packet,raw,ids,length)
            except DeviceStatusError as exc:return None,bus.sdk.COMM_SUCCESS,exc.error,dict(completed_ids=completed,device_error_id=exc.motor,requested_ids=ids,fast_sync_read=True)
            except ValueError as exc:return None,bus.sdk.COMM_RX_CORRUPT,0,dict(completed_ids=completed,decode_error=str(exc),requested_ids=ids,fast_sync_read=True)
            for motor in ids:
                data=decoded[motor]
                for name,addr,size in fields:
                    start=addr-address
                    if start<0 or start+size>length:raise ValueError('Field outside complete health span')
                    values[name][motor-1]=int.from_bytes(data[start:start+size],'little')
                completed.append(motor)
        finally:bus.port.is_using=False
    if time.perf_counter()>=deadline:raise RuntimeError('Complete Fast READ exceeded shared200ms; no target advance')
    if completed!=list(range(1,17)):raise RuntimeError('Incomplete Fast READ snapshot; no target advance')
    bus.report.setdefault('fast_health_complete_reads',[]).append(dict(elapsed_ms=1000*(time.perf_counter()-began),groups=4,complete_ids=completed))
    return values,bus.sdk.COMM_SUCCESS,0,{}
