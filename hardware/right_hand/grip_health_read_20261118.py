"""Smaller ordinary Sync READ bursts; unchanged full-state/CRC/deadline guards."""
import time
import numpy as np

POLICY=dict(version='paired_standard_sync_read_QPC_v1_20261118',instruction=0x82,
    group_size=2,packet_window_ms=40.,sample_budget_ms=200,full16_required=True,
    each_status_error_checked=True,SDK_CRC_and_stuffing_preserved=True,
    partial_snapshot_reused=False,goal_writes=0,host_read_timeout_seconds=.001,
    maximum_failed_packet_traces=64)

def install(bus):
    # Host serial pacing only. No EEPROM, motor RAM or USB driver settings.
    bus.port.ser.timeout=.001
    original=bus.packet.rxPacket
    def receive(port,fast):
        began=time.perf_counter();raw,comm=original(port,fast)
        if comm!=bus.sdk.COMM_SUCCESS:
            events=bus.report.setdefault('failed_raw_packets',[])
            if len(events)<64:
                data=bytes(raw)
                declared=7+int.from_bytes(data[5:7],'little') if len(data)>=7 and data[:4]==b'\xff\xff\xfd\x00' else None
                complete=declared is not None and len(data)==declared
                events.append(dict(comm=int(comm),received_bytes=len(data),declared_bytes=declared,
                    length_complete=complete,raw_hex=data.hex(),elapsed_ms=1000*(time.perf_counter()-began),
                    crc_matches=bool(bus.packet.updateCRC(0,list(data[:-2]),len(data)-2)==int.from_bytes(data[-2:],'little')) if complete else None))
        return raw,comm
    bus.packet.rxPacket=receive
    bus.report['active_health_read_policy']=dict(POLICY)

def block_once(bus,address,length,fields):
    if (address,length)!=(64,83):raise ValueError('Reader limited to full health scope')
    if any(not address<=at or at+size>address+length for _,at,size in fields):
        raise ValueError('Field outside health range')
    began=time.perf_counter();deadline=min(began+.2,getattr(bus,'_horizontal_sample_deadline',float('inf')))
    values={name:np.empty(16,dtype=np.int64) for name,_,_ in fields};completed=[]
    for first in range(1,17,2):
        ids=[first,first+1]
        detail=dict(completed_ids=completed.copy(),requested_ids=ids,health_reply_group_size=2,
                    fast_sync_read=False,standard_sync_read=True)
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
                if comm!=bus.sdk.COMM_SUCCESS or error or len(data)!=length:
                    detail.update(completed_ids=completed.copy(),first_incomplete_id=motor,
                        device_error_id=motor if error else None,receive_window_ms=window*1000,
                        received_data_bytes=len(data or []),expected_data_bytes=length)
                    return None,comm if comm else (0 if error else bus.sdk.COMM_RX_CORRUPT),error,detail
                if time.perf_counter()>=deadline:raise RuntimeError('Complete Sync READ exceeded shared200ms')
                for name,at,size in fields:
                    values[name][motor-1]=int.from_bytes(bytes(data[at-address:at-address+size]),'little')
                completed.append(motor)
        finally:bus.port.is_using=False
    if completed!=list(range(1,17)):raise RuntimeError('Incomplete Sync READ snapshot')
    bus.report['active_health_read_policy']=dict(POLICY)
    bus.report.setdefault('standard_health_complete_reads',[]).append(dict(elapsed_ms=1000*(time.perf_counter()-began),complete_ids=completed,group_size=2))
    return values,bus.sdk.COMM_SUCCESS,0,{}
