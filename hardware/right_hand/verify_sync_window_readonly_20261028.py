"""Exercise the new host receive window with camera; PING/READ only."""
import datetime as dt,json,time,sys,statistics
from pathlib import Path
from types import SimpleNamespace
from horizontal_sync_receive_window_20261028 import block_once
ROOT=Path(__file__).resolve().parent
def main():
    if '--read' not in sys.argv:print('CHECK ONLY: no devices opened.');return
    import dynamixel_sdk as sdk
    from camera_stream import CameraStream
    out=ROOT/('sync_window_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
    report=dict(status='started',reads=0,errors=[],camera_frames=0,motor_writes=0,baud=3000000)
    port=sdk.PortHandler('COM7');packet=sdk.PacketHandler(2.0);camera=None;opened=False;times=[]
    bus=SimpleNamespace(sdk=sdk,port=port,packet=packet,report=report)
    try:
        camera=CameraStream(report)
        if not port.openPort() or not port.setBaudRate(3000000):raise RuntimeError('Cannot open existing3Mbps')
        opened=True
        for i in range(1,17):
            model,comm,error=packet.ping(port,i)
            if comm or error or model!=1240:raise RuntimeError(f'ID{i} PING failed')
        fields=[('torque',64,1),('error',70,1),('watchdog',98,1),('voltage',144,2),('temperature',146,1)]
        for index in range(1000):
            began=time.perf_counter();state,comm,error,detail=block_once(bus,64,83,fields);elapsed=(time.perf_counter()-began)*1000
            if comm or error:
                report['errors'].append(dict(index=index,comm=int(comm),error=int(error),elapsed_ms=elapsed,detail=detail));continue
            if elapsed>200:raise RuntimeError('Whole read exceeds unchanged200ms budget')
            if any(state[k].any() for k in ('torque','error','watchdog')):raise RuntimeError('Not healthy OFF; no writes sent')
            if min(state['voltage'])<38 or max(state['voltage'])>65 or max(state['temperature'])>45:raise RuntimeError('OFF voltage/temperature abnormal')
            times.append(elapsed);report['reads']+=1
            if camera.read_packet() is not None:report['camera_frames']+=1
        report['status']='readonly_window_check_complete'
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        if opened:port.closePort()
        if camera is not None:camera.release()
        if times:report.update(median_ms=statistics.median(times),max_ms=max(times))
        (out/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=out.name,status=report['status'],full_reads=report['reads'],errors=len(report['errors']),
        median_ms=report.get('median_ms'),max_ms=report.get('max_ms'),camera_frames=report['camera_frames'],motor_writes=0)))
if __name__=='__main__':main()
