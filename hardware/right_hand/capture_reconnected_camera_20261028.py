"""One fresh unattended camera observation; no motor imports or APIs."""
import datetime as dt,json,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def main():
    from camera_stream import CameraStream
    from camera_frame_signal import frame_signal
    out=ROOT/('reconnected_camera_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
    report=dict(status='started',motor_writes=0,camera_only=True);camera=None
    try:
        import cv2 as cv
        camera=CameraStream(report);began=time.perf_counter();packet=None
        while time.perf_counter()-began<3.:
            fresh=camera.read_packet()
            if fresh is not None and frame_signal(fresh[1])['usable']:packet=fresh
            time.sleep(.03)
        if packet is None:raise RuntimeError('No fresh usable webcam frame')
        ok,data=cv.imencode('.jpg',packet[1]);assert ok
        (out/'hand.jpg').write_bytes(data.tobytes());report.update(status='fresh_camera_frame_saved',image='hand.jpg',image_signal=frame_signal(packet[1]),camera_stamp=packet[0],geometry_calibration_verified=False)
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        if camera is not None:camera.release()
        (out/'result.json').write_bytes((json.dumps(report,indent=2)+'\n').encode())
    print(json.dumps(dict(folder=out.name,status=report['status'],error=report.get('error'),image=report.get('image'),motor_writes=0)))
if __name__=='__main__':main()
