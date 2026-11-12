"""Run the exact grouped full-health decoder at1Mbps; PING/READ only."""
import argparse,datetime as dt,json,statistics,time
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parent
def main():
    p=argparse.ArgumentParser();p.add_argument('--read',action='store_true');args=p.parse_args()
    if not args.read:print('CHECK ONLY. --read is PING/READ only; no torque/goal/setting writes.');return
    import dynamixel_sdk as sdk
    from horizontal_sync_receive_window_20261028 import block_once
    from horizontal_serial_link_20261028 import metadata
    from camera_stream import CameraStream
    cfg=metadata();folder=ROOT/('production_link_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report=dict(status='started',baud=cfg['baud'],motor_writes=0,full_health_group_size=1,reads=0,errors=[],camera_frames=0,last_snapshots=[])
    port=sdk.PortHandler('COM7');packet=sdk.PacketHandler(2.0);opened=False;camera=None;times=[]
    bus=SimpleNamespace(sdk=sdk,port=port,packet=packet,report=report)
    fields=[('torque',64,1),('error',70,1),('d_gain',80,2),('i_gain',82,2),('p_gain',84,2),('watchdog',98,1),
        ('current_cap',102,2),('acceleration',108,4),('velocity',112,4),('goal',116,4),('current',126,2),
        ('position',132,4),('voltage',144,2),('temperature',146,1)]
    try:
        camera=CameraStream(report)
        if not port.openPort() or not port.setBaudRate(cfg['baud']):raise RuntimeError('Cannot open verified1Mbps')
        opened=True
        for motor in range(1,17):
            model,comm,error=packet.ping(port,motor)
            if comm or error or model!=1240:raise RuntimeError(f'ID{motor} PING failed')
        for index in range(1000):
            began=time.perf_counter();state,comm,error,detail=block_once(bus,64,83,fields);elapsed=(time.perf_counter()-began)*1000
            if comm or error:
                report['errors'].append(dict(index=index,comm=int(comm),detail=detail));time.sleep(.02);continue
            if any(state[k].any() for k in ('torque','error','watchdog')):raise RuntimeError('Not healthy OFF')
            if min(state['voltage'])<38 or max(state['voltage'])>65 or max(state['temperature'])>45:raise RuntimeError('OFF health outside checks')
            report['last_snapshots']=(report['last_snapshots']+[{k:v.tolist() for k,v in state.items()}])[-3:]
            times.append(elapsed);report['reads']+=1
            if camera.read_packet() is not None:report['camera_frames']+=1
        report['status']='production_decoder_readonly_complete'
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        if opened:port.closePort()
        if camera is not None:camera.release()
        if times:report.update(median_ms=statistics.median(times),maximum_ms=max(times))
        (folder/'result.json').write_bytes((json.dumps(report,indent=2)+'\n').encode())
    print(json.dumps(dict(folder=folder.name,status=report['status'],error=report.get('error'),reads=report['reads'],
        errors=len(report['errors']),camera_frames=report['camera_frames'],median_ms=report.get('median_ms'),maximum_ms=report.get('maximum_ms'),motor_writes=0)))
if __name__=='__main__':main()
