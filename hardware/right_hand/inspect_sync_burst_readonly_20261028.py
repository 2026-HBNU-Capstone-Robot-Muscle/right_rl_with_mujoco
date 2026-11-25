"""Compare reply burst sizes with wire capture; only PING and Sync READ."""
import argparse,datetime as dt,json,statistics,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def decode_wire(data,packet):
    frames=[];position=0;skipped=0
    while position+7<=len(data):
        start=data.find(b'\xff\xff\xfd\x00',position)
        if start<0:break
        skipped+=start-position;length=int.from_bytes(data[start+5:start+7],'little');end=start+7+length
        if not 4<=length<=1024 or end>len(data):
            frames.append(dict(offset=start,id=data[start+4],length_field=length,incomplete=True,available=len(data)-start));break
        frame=data[start:end];actual=int.from_bytes(frame[-2:],'little');computed=packet.updateCRC(0,list(frame[:-2]),len(frame)-2)
        frames.append(dict(offset=start,id=frame[4],length=len(frame),crc_ok=actual==computed,
            crc_received=actual,crc_expected=computed,stuffing_patterns=frame.count(b'\xff\xff\xfd\xfd')))
        position=end
    return dict(wire_bytes=len(data),skipped_bytes=skipped,frames=frames,unparsed_tail_bytes=len(data)-position)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--read',action='store_true');parser.add_argument('--baud',type=int,choices=(1000000,3000000),default=3000000);parser.add_argument('--compare-camera',action='store_true');parser.add_argument('--cycles',type=int,default=200,choices=range(50,1001));parser.add_argument('--group-size',type=int,choices=(1,4,8,16));parser.add_argument('--win32-errors',action='store_true');args=parser.parse_args()
    if not args.read:print('CHECK ONLY. --read sends PING/Sync READ only.');return
    import dynamixel_sdk as sdk
    from camera_stream import CameraStream
    out=ROOT/('sync_burst_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
    report=dict(status='started',baud=args.baud,motor_writes=0,torque_changes=0,blocks=[],bad_transactions=[],camera_frames=0)
    restore_comm=None
    if args.win32_errors:
        import ctypes,serial.win32 as serial_win32
        restore_comm=serial_win32.ClearCommError;report['driver_errors']=[];report['maximum_driver_input_bytes']=0
        def trace_comm(handle,flags,stat):
            ok=restore_comm(handle,flags,stat)
            if ok:
                value=ctypes.cast(flags,ctypes.POINTER(serial_win32.DWORD)).contents.value
                queued=ctypes.cast(stat,ctypes.POINTER(serial_win32.COMSTAT)).contents.cbInQue
                report['maximum_driver_input_bytes']=max(report['maximum_driver_input_bytes'],int(queued))
                if value:report['driver_errors'].append(dict(flags=int(value),input_bytes=int(queued),t=time.perf_counter()))
            return ok
        serial_win32.ClearCommError=trace_comm
    port=sdk.PortHandler('COM7');packet=sdk.PacketHandler(2.0);camera=None;opened=False
    original_read=port.readPort;original_write=port.writePort;wire=bytearray();chunks=[]
    def read(n):
        data=original_read(n)
        if data:wire.extend(data);chunks.append(dict(t=time.perf_counter(),bytes=len(data)))
        return data
    def write(data):
        if len(data)<8 or data[7] not in (1,0x82):raise RuntimeError('Non-PING/Sync READ transmission forbidden')
        return original_write(data)
    try:
        if not args.compare_camera:camera=CameraStream(report)
        if not port.openPort() or not port.setBaudRate(args.baud):raise RuntimeError('Existing COM7 speed unavailable')
        opened=True;port.readPort=read;port.writePort=write
        for motor in range(1,17):
            model,comm,error=packet.ping(port,motor)
            if comm or error or model!=1240:raise RuntimeError(f'ID{motor} PING failed')
        conditions=[(False,16),(True,16)] if args.compare_camera else [(True,args.group_size)] if args.group_size else [(True,n) for n in (16,8,4,4,8,16)]
        for block_index,(with_camera,size) in enumerate(conditions):
            if with_camera and camera is None:camera=CameraStream(report)
            block=dict(group_size=size,camera_on=with_camera,snapshots_requested=args.cycles,complete=0,errors=[],read_ms=[])
            report['blocks'].append(block)
            for index in range(args.cycles):
                began=time.perf_counter();states={};failed=False
                for start in range(1,17,size):
                    ids=list(range(start,min(start+size,17)));group=sdk.GroupSyncRead(port,packet,64,83)
                    wire.clear();chunks.clear();comm=0
                    try:
                        for motor in ids:group.addParam(motor)
                        comm=group.txPacket()
                        if not comm:port.setPacketTimeoutMillis(80.);comm=group.rxPacket()
                        if comm or not all(group.isAvailable(i,64,83) for i in ids):
                            # Record delayed bytes too, before a later SDK TX clears them.
                            time.sleep(.02)
                            available=port.getBytesAvailable()
                            if available:port.readPort(available)
                            event=dict(block=block_index,index=index,ids=ids,comm=int(comm),
                                elapsed_ms=1000*(time.perf_counter()-began),decode=decode_wire(bytes(wire),packet))
                            block['errors'].append(event)
                            if len(report['bad_transactions'])<24:
                                name=f'bad_{len(report["bad_transactions"]):02d}.bin';(out/name).write_bytes(wire)
                                report['bad_transactions'].append(dict(event,wire_file=name,chunks=chunks.copy()))
                            failed=True;time.sleep(.02);break
                        for motor in ids:
                            state=dict(torque=group.getData(motor,64,1),error=group.getData(motor,70,1),
                                watchdog=group.getData(motor,98,1),voltage=group.getData(motor,144,2),temperature=group.getData(motor,146,1))
                            if state['torque'] or state['error'] or state['watchdog']:raise RuntimeError(f'ID{motor} not healthy OFF; no writes sent')
                            if not 38<=state['voltage']<=65 or state['temperature']>45:raise RuntimeError('OFF voltage/temperature abnormal')
                            states[motor]=state
                    finally:group.clearParam()
                if not failed:
                    if len(states)!=16:raise RuntimeError('Incomplete combined snapshot')
                    block['complete']+=1;block['read_ms'].append(1000*(time.perf_counter()-began))
                if camera is not None and camera.read_packet() is not None:report['camera_frames']+=1
            values=block.pop('read_ms')
            if values:block.update(median_ms=statistics.median(values),maximum_ms=max(values))
            print(json.dumps(dict(group_size=size,camera_on=with_camera,complete=block['complete'],errors=len(block['errors']),median_ms=block.get('median_ms'))),flush=True)
        report['status']='readonly_burst_comparison_complete'
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        if opened:port.closePort()
        if camera is not None:camera.release()
        if restore_comm is not None:serial_win32.ClearCommError=restore_comm
        (out/'result.json').write_bytes((json.dumps(report,indent=2)+'\n').encode())
    print(json.dumps(dict(folder=out.name,status=report['status'],error=report.get('error'),bad_transactions=len(report['bad_transactions']),camera_frames=report['camera_frames'],motor_writes=0)),flush=True)
if __name__=='__main__':main()
