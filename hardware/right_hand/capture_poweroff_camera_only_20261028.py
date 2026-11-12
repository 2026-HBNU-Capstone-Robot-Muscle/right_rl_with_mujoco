"""Camera-only inspection. No motor libraries, ports, reads or writes."""
import datetime as dt,json,time,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def main():
    if '--capture' not in sys.argv:print('CHECK ONLY: no devices opened.');return
    import cv2 as cv
    from camera_stream import CameraStream
    out=ROOT/('poweroff_connection_camera_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
    report=dict(status='started',motor_port_opened=False,motor_commands=0,frames=[]);camera=None
    try:
        camera=CameraStream(report);last=time.monotonic();deadline=last+5
        while len(report['frames'])<3:
            if time.monotonic()>deadline:raise RuntimeError('No three fresh frames in5s')
            packet=camera.read_packet()
            if packet is None:time.sleep(.005);continue
            stamp,frame=packet
            if stamp<=last or time.monotonic()-stamp>.3:continue
            name=f'frame_{len(report["frames"]):02d}.jpg';ok,data=cv.imencode('.jpg',frame)
            if not ok:raise RuntimeError('Cannot encode camera frame')
            (out/name).write_bytes(data.tobytes());report['frames'].append(dict(image=name,read_return_monotonic=stamp));last=stamp
        report['status']='camera_only_frames_saved'
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        if camera is not None:camera.release()
        (out/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=out.name,status=report['status'],frames=len(report['frames']),error=report.get('error'),motor_commands=0)))
if __name__=='__main__':main()
