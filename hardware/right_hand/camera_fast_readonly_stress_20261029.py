"""Isolate camera/USB load with torque OFF; never sends motion/settings writes."""
import argparse,datetime as dt,json,time,statistics
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parent
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--read',action='store_true');a=parser.parse_args()
    if not a.read:print('No devices opened. --read permits camera and PING/READ only.');return
    import dynamixel_sdk as sdk
    from serial.tools.list_ports import comports
    from fast_health_read_20261029 import block_once
    from calibrated_precise_clock_20261028 import install
    from observed_book_model_policy import healthy_off
    from camera_stream import CameraStream
    out=ROOT/('camera_fast_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
    report=dict(status='started',servo_writes=0,torque_changes=0,instruction_counts={},blocks=[]);port=camera=None
    try:
        matches=[p for p in comports() if p.vid==1027 and p.pid==24596 and p.serial_number=='FTBENPSAA']
        if len(matches)!=1:raise RuntimeError('Known adapter missing/ambiguous')
        port=sdk.PortHandler(matches[0].device);packet=sdk.PacketHandler(2.)
        if not port.openPort() or not port.setBaudRate(1000000):raise RuntimeError('Cannot open known1Mbps')
        original=port.writePort
        def readonly(data):
            if len(data)<8 or data[7] not in (1,2,0x8a):raise RuntimeError('Motor write instruction blocked')
            if data[7]==0x8a and (data[8:12]!=[64,0,83,0] or list(data[12:-2]) not in [list(range(1,9)),list(range(9,17))]):raise RuntimeError('READ outside validated health scope')
            key=str(data[7]);report['instruction_counts'][key]=report['instruction_counts'].get(key,0)+1;return original(data)
        port.writePort=readonly;bus=SimpleNamespace(sdk=sdk,packet=packet,port=port,report=report);install(bus)
        for motor in range(1,17):
            model,comm,error=packet.ping(port,motor)
            if comm or error or model!=1240:raise RuntimeError('PING failed')
            for address,expected in ((8,3),(11,5),(64,0),(70,0),(98,0)):
                value,comm,error=packet.read1ByteTxRx(port,motor,address)
                if comm or error or value!=expected:raise RuntimeError('Unexpected mode/OFF state')
        fields=[('torque',64,1),('error',70,1),('d_gain',80,2),('i_gain',82,2),('p_gain',84,2),('watchdog',98,1),('current_cap',102,2),('acceleration',108,4),('velocity',112,4),('goal',116,4),('current',126,2),('position',132,4),('voltage',144,2),('temperature',146,1)]
        for name,count in (('camera_OFF',100),('camera_ON',500)):
            if name=='camera_ON':camera=CameraStream(report,timeout=10.)
            row=dict(name=name,requested=count,complete=0,errors=[],read_ms=[],camera_frames=0);report['blocks'].append(row)
            for index in range(count):
                began=time.perf_counter();bus._horizontal_sample_deadline=began+.2
                state,comm,error,detail=block_once(bus,64,83,fields)
                elapsed=1000*(time.perf_counter()-began);row['read_ms'].append(elapsed)
                if comm or error:
                    row['errors'].append(dict(index=index,comm=comm,error=error,elapsed_ms=elapsed,detail=detail))
                else:healthy_off(state);row['complete']+=1
                if camera is not None:
                    camera.check_preview('READ ONLY');frame=camera.read_packet()
                    if frame is not None:row['camera_frames']+=1
                remaining=.05-(time.perf_counter()-began)
                if remaining>0:time.sleep(remaining)
            row['median_ms']=statistics.median(row['read_ms']);row['maximum_ms']=max(row['read_ms'])
            print(json.dumps({k:v for k,v in row.items() if k!='read_ms'}),flush=True)
        report['status']='camera_fast_readonly_complete'
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        if port is not None:port.closePort()
        if camera is not None:camera.release()
        (out/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=out.name,status=report['status'],error=report.get('error'),servo_writes=0,torque_changes=0)))
if __name__=='__main__':main()
