"""READ/PING-only packet stress; no setting, goal or torque writes."""
import datetime as dt,json,time,sys,statistics
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def main():
    if '--read' not in sys.argv:print('CHECK ONLY. --read uses PING/READ only.');return
    import dynamixel_sdk as sdk
    folder=ROOT/('serial_packet_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report=dict(baud=3000000,requested_reads=1000,reads=0,errors=[],bad_packets=[],servo_writes=0,torque_changes=0,
        camera_concurrent='--camera' in sys.argv,camera_frames=0)
    port=sdk.PortHandler('COM7');packet=sdk.PacketHandler(2.0);opened=False;durations=[];camera=None
    try:
        if report['camera_concurrent']:
            from camera_stream import CameraStream
            camera=CameraStream(report)
        if not port.openPort() or not port.setBaudRate(3000000):raise RuntimeError('Port unavailable')
        opened=True
        for motor in range(1,17):
            model,comm,error=packet.ping(port,motor)
            if comm or error or model!=1240:raise RuntimeError(f'ID{motor} PING failed')
        original=packet.rxPacket
        def trace(p,*args):
            raw,comm=original(p,*args)
            if comm and len(report['bad_packets'])<20:
                report['bad_packets'].append(dict(comm=int(comm),length=len(raw),prefix=bytes(raw[:40]).hex()))
            return raw,comm
        packet.rxPacket=trace
        for index in range(1000):
            group=sdk.GroupSyncRead(port,packet,64,83)
            try:
                for motor in range(1,17):group.addParam(motor)
                began=time.perf_counter();comm=group.txRxPacket();elapsed=(time.perf_counter()-began)*1000
                if comm or not all(group.isAvailable(i,64,83) for i in range(1,17)):
                    report['errors'].append(dict(index=index,comm=int(comm),elapsed_ms=elapsed))
                    time.sleep(.02);continue
                durations.append(elapsed);report['reads']+=1
                if any(group.getData(i,a,1) for i in range(1,17) for a in (64,70,98)):
                    raise RuntimeError('Torque/error/watchdog changed; stopped without writes')
                if any(not 38<=group.getData(i,144,2)<=65 for i in range(1,17)):
                    raise RuntimeError('Abnormal OFF voltage')
                if camera is not None:
                    frame=camera.read_packet()
                    if frame is not None:
                        report['camera_frames']+=1
                        if report['camera_frames']==1:
                            import cv2 as cv
                            ok,encoded=cv.imencode('.jpg',frame[1])
                            if not ok:raise RuntimeError('Cannot save concurrent camera frame')
                            (folder/'camera_first.jpg').write_bytes(encoded.tobytes())
            finally:group.clearParam()
        report['status']='readonly_stress_complete'
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        if opened:port.closePort()
        if camera is not None:camera.release()
        if durations:report.update(median_ms=statistics.median(durations),max_ms=max(durations))
        (folder/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=folder.name,status=report['status'],full_reads=report['reads'],errors=len(report['errors']),median_ms=report.get('median_ms'),camera_frames=report['camera_frames'],motor_writes=0)),flush=True)
if __name__=='__main__':main()
