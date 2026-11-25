"""Matched-rate camera/bus observation after physical USB port exchange.

Only PING, READ and Sync READ instructions may reach the USB adapter.
"""
import argparse,datetime as dt,json,statistics,time
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parent
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--read',action='store_true');args=parser.parse_args()
    if not args.read:print('CHECK ONLY: no camera/port opened; --read is PING/READ only.');return
    import dynamixel_sdk as sdk
    from serial.tools.list_ports import comports
    from camera_stream import CameraStream
    from camera_frame_signal import frame_signal
    from horizontal_sync_receive_window_20261028 import block_once
    out=ROOT/('usb_swap_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
    report=dict(status='started',timestamp=dt.datetime.now().astimezone().isoformat(),baud=1000000,
        motor_writes=0,torque_changes=0,sample_period_s=.075,requested_duration_s=60.,
        reads=0,errors=[],camera_frames=0,usable_frames=0,blank_frames=0,camera_gaps_s=[],
        camera_samples=[],last_snapshots=[],individual_checks=[])
    port=None;camera=None;times=[];opened=False;last_frame=None;first_frame=None;last_saved=-1.
    def observe_camera():
        nonlocal first_frame,last_frame,last_saved
        packet=camera.read_packet()
        if packet is None:return
        stamp,frame=packet;signal=frame_signal(frame)
        report['camera_frames']+=1;report['usable_frames']+=int(signal['usable']);report['blank_frames']+=int(not signal['usable'])
        if last_frame is not None:report['camera_gaps_s'].append(stamp-last_frame)
        if first_frame is None:first_frame=stamp
        last_frame=stamp
        if stamp-first_frame-last_saved>=10 or not signal['usable']:
            import cv2 as cv
            filename=f'camera_{report["camera_frames"]:05d}.jpg';ok,encoded=cv.imencode('.jpg',frame)
            if ok:(out/filename).write_bytes(encoded.tobytes())
            report['camera_samples'].append(dict(t=stamp-first_frame,file=filename,signal=signal))
            last_saved=stamp-first_frame
    try:
        matches=[p for p in comports() if p.vid==1027 and p.pid==24596 and p.serial_number=='FTBENPSAA']
        if len(matches)!=1:raise RuntimeError('Expected exactly one known motor USB adapter')
        device=matches[0].device;report['device']=device
        port=sdk.PortHandler(device);packet=sdk.PacketHandler(2.0)
        if not port.openPort() or not port.setBaudRate(1000000):raise RuntimeError('Cannot open existing1Mbps')
        opened=True
        original_write=port.writePort
        def readonly_write(data):
            if len(data)<8 or data[7] not in (1,2,0x82):raise RuntimeError('Non-read motor instruction forbidden')
            return original_write(data)
        port.writePort=readonly_write
        for motor in range(1,17):
            model,comm,error=packet.ping(port,motor)
            if comm or error or model!=1240:raise RuntimeError(f'ID{motor} PING failed at1Mbps')
            checks={}
            for address,expected in ((8,3),(11,5),(64,0),(70,0),(98,0)):
                value,comm,error=packet.read1ByteTxRx(port,motor,address)
                if comm or error or value!=expected:raise RuntimeError(f'ID{motor} unexpected READ{address}: value={value} comm={comm} error={error}')
                checks[str(address)]=int(value)
            report['individual_checks'].append(dict(motor_id=motor,model=model,registers=checks))
        camera=CameraStream(report);bus=SimpleNamespace(sdk=sdk,port=port,packet=packet,report=report)
        fields=[('torque',64,1),('error',70,1),('d_gain',80,2),('i_gain',82,2),('p_gain',84,2),('watchdog',98,1),
            ('current_cap',102,2),('acceleration',108,4),('velocity',112,4),('goal',116,4),('current',126,2),
            ('position',132,4),('voltage',144,2),('temperature',146,1)]
        began=time.perf_counter();last_print=0.
        while time.perf_counter()-began<60.:
            cycle=time.perf_counter();observe_camera()
            state,comm,error,detail=block_once(bus,64,83,fields);elapsed=(time.perf_counter()-cycle)*1000
            if comm or error:
                report['errors'].append(dict(t=cycle-began,comm=int(comm),detail=detail));time.sleep(.02)
            else:
                if any(state[k].any() for k in ('torque','error','watchdog')):raise RuntimeError('Not healthy OFF; no writes sent')
                if min(state['voltage'])<38 or max(state['voltage'])>65 or max(state['temperature'])>45:raise RuntimeError('OFF voltage/temperature abnormal')
                report['reads']+=1;times.append(elapsed)
                report['last_snapshots']=(report['last_snapshots']+[{k:v.tolist() for k,v in state.items()}])[-3:]
            while time.perf_counter()-cycle<.075:
                observe_camera();time.sleep(.002)
            if time.perf_counter()-began-last_print>=15.:
                last_print=time.perf_counter()-began
                print(json.dumps(dict(elapsed_s=round(last_print),reads=report['reads'],errors=len(report['errors']),frames=report['camera_frames'],blank=report['blank_frames'])),flush=True)
        observe_camera();report['status']='swapped_usb_readonly_observation_complete'
        if first_frame is not None:report['camera_observed_duration_s']=last_frame-first_frame
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc) or 'Interrupted')
    finally:
        if opened:port.closePort()
        if camera is not None:camera.release()
        if times:report.update(median_read_ms=statistics.median(times),maximum_read_ms=max(times))
        gaps=report['camera_gaps_s'];report['maximum_frame_gap_s']=max(gaps,default=None)
        (out/'result.json').write_bytes((json.dumps(report,indent=2)+'\n').encode())
    print(json.dumps(dict(folder=out.name,status=report['status'],error=report.get('error'),device=report.get('device'),
        reads=report['reads'],errors=len(report['errors']),frames=report['camera_frames'],blank_frames=report['blank_frames'],
        frame_gap_max_s=report['maximum_frame_gap_s'],median_read_ms=report.get('median_read_ms'),maximum_read_ms=report.get('maximum_read_ms'),motor_writes=0)),flush=True)
if __name__=='__main__':main()
