"""Camera + PING/READ only; no motor write APIs or automatic movement."""
import datetime as dt,json,sys,time
from pathlib import Path

def main():
    if '--capture' not in sys.argv:print('CHECK ONLY: no devices opened.');return
    import cv2 as cv
    import dynamixel_sdk as sdk
    from camera_stream import CameraStream
    from leap_camera_readonly_check import ReadonlyBus,validate_off
    root=Path(__file__).resolve().parent;out=root/('rewired_contact_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
    report=dict(status='started',motor_writes=0,torque_or_setting_writes=0,frames=[]);camera=bus=None
    try:
        camera=CameraStream(report);bus=ReadonlyBus(sdk,'COM7');bus.open()
        report['before']=bus.snapshot();validate_off(report['before']);cutoff=time.monotonic();deadline=cutoff+5
        while len(report['frames'])<5:
            if time.monotonic()>deadline:raise RuntimeError('No five fresh frames in5s')
            packet=camera.read_packet()
            if packet is None:time.sleep(.005);continue
            stamp,frame=packet
            if not cutoff<stamp<=time.monotonic() or time.monotonic()-stamp>.3:continue
            name='frame_%02d.jpg'%len(report['frames']);ok,encoded=cv.imencode('.jpg',frame)
            if not ok:raise RuntimeError('Could not encode frame')
            (out/name).write_bytes(encoded.tobytes());report['frames'].append(dict(image=name,camera_driver_return_monotonic=stamp));cutoff=stamp
        report['after']=bus.snapshot();validate_off(report['after']);drift=[abs(a-b) for a,b in zip(report['before']['position_ticks'],report['after']['position_ticks'])]
        report.update(status='fresh_camera_and_off_readings_saved',maximum_endpoint_drift_ticks=max(drift),stationary_endpoints=max(drift)<=3,geometry_calibration_verified=False,camera_metadata=camera.meta)
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        if bus is not None:bus.close()
        if camera is not None:camera.release()
        (out/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=out.name,status=report['status'],maximum_endpoint_drift_ticks=report.get('maximum_endpoint_drift_ticks'),frames=len(report['frames']),error=report.get('error'),motor_writes=0)))
if __name__=='__main__':main()
