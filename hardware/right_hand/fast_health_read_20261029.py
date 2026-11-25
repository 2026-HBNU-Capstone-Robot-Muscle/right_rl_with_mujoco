"""ROBOTIS Fast Sync Read, two8-device replies below the SDK1024-byte cap.

Official protocol: https://emanual.robotis.com/docs/en/dxl/protocol2/#fast-sync-read-0x8a
X330 firmware>=46; no register mapping, EEPROM, torque or goal writes.
All16 complete responses, exact IDs, every status error and final CRC required.
"""
import time
import numpy as np
POLICY=dict(version='strict_fast_sync_read8_QPC_v1_20261029',instruction=0x8a,address=64,length=83,
    group_size=8,snapshot_ids=list(range(1,17)),packet_window_ms=40.,overall_health_sample_budget_ms=200,
    required_x330_firmware=46,maximum_reply_bytes=704,sdk_reply_limit_unchanged=1024,
    no_byte_stuffing_on_fast_status=True,all_status_errors_checked=True,exact_unique_ordered_ids=True,
    final_crc_checked=True,all16_fresh_before_return=True,partial_snapshot_reused=False,
    ordinary_OFF_readback_unchanged=True,motor_register_writes=0)

class DeviceStatusError(RuntimeError):
    def __init__(self,motor,error):self.motor=motor;self.error=error;super().__init__(f'Fast READ ID{motor} status error{error}')

def decode_status(packet,raw,ids,length):
    """Decode the complete CRC-protected frame, without changing byte stuffing."""
    raw=bytes(raw);ids=tuple(ids)
    if not ids or len(ids)>8 or len(set(ids))!=len(ids) or any(not 1<=i<=16 for i in ids):raise ValueError('Invalid fast health IDs')
    expected=8+len(ids)*(length+4)
    if len(raw)!=expected or raw[:5]!=b'\xff\xff\xfd\x00\xfe' or raw[7]!=0x55 or int.from_bytes(raw[5:7],'little')+7!=len(raw):raise ValueError('Fast status header/length mismatch')
    if packet.updateCRC(0,list(raw[:-2]),len(raw)-2)!=int.from_bytes(raw[-2:],'little'):raise ValueError('Fast status CRC mismatch')
    values={}
    for n,motor in enumerate(ids):
        offset=8+n*(length+4);error,actual=raw[offset:offset+2]
        if actual!=motor:raise ValueError('Fast status missing, duplicate or reordered ID')
        if error:raise DeviceStatusError(actual,error)
        values[actual]=raw[offset+2:offset+2+length]
    return values

def block_once(bus,address,length,fields):
    if (address,length)!=(64,83):raise ValueError('Fast read limited to full health scope')
    began=time.perf_counter();deadline=min(began+.2,getattr(bus,'_horizontal_sample_deadline',float('inf')))
    values={name:np.empty(16,dtype=np.int64) for name,_,_ in fields};completed=[]
    bus.report['fast_health_read_policy']=dict(POLICY)
    for first in (1,9):
        ids=list(range(first,first+8));window=40.
        try:
            getattr(bus,'cancel_callback',lambda:None)()
            if time.perf_counter()>=deadline:raise RuntimeError('Shared200ms expired before Fast READ; no target advance')
            comm=bus.packet.syncReadTx(bus.port,address,length,ids,len(ids),True)
            if comm!=bus.sdk.COMM_SUCCESS:return None,comm,0,dict(completed_ids=completed,requested_ids=ids,health_reply_group_size=8,fast_sync_read=True)
            window=min(40.,1000*(deadline-time.perf_counter()))
            if window<=0:raise RuntimeError('Shared200ms expired after Fast READ transmit; no target advance')
            bus.port.setPacketTimeoutMillis(window)
            raw,comm=bus.packet.rxPacket(bus.port,True)
            if comm!=bus.sdk.COMM_SUCCESS:return None,comm,0,dict(completed_ids=completed,first_incomplete_id=first,requested_ids=ids,receive_window_ms=window,health_reply_group_size=8,fast_sync_read=True,received_packet_bytes=len(raw or []),expected_packet_bytes=704)
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
    bus.report.setdefault('fast_health_complete_reads',[]).append(dict(elapsed_ms=1000*(time.perf_counter()-began),groups=2,complete_ids=completed))
    return values,bus.sdk.COMM_SUCCESS,0,{}
