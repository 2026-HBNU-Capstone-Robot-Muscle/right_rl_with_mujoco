"""Bounded automatic camera snapshot. No motor API or setting writes."""
import datetime as dt,json,time,hashlib
from pathlib import Path
from camera_stream import CameraStream
ROOT=Path(__file__).resolve().parent
def main():
    import cv2
    out=ROOT/('contact_view_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
    r=dict(status='started',motor_api_used=False,motor_goals_sent=0,frames=[]);stream=None
    try:
        stream=CameraStream(r,timeout=10.);began=time.monotonic();count=0
        while time.monotonic()-began<6 and count<30:
            packet=stream.read_packet()
            if packet is None:time.sleep(.01);continue
            stamp,frame=packet;count+=1
            if count in (10,20,30):
                path=out/f'frame_{count:03d}.jpg';ok=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,98])[1]
                path.write_bytes(ok.tobytes());r['frames'].append(dict(file=path.name,stamp=stamp,mean=float(frame.mean()),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        r.update(status='frames_saved' if r['frames'] else 'no_saved_fresh_frames',fresh_frames=count)
    except (Exception,KeyboardInterrupt) as exc:r.update(status='stopped',error=str(exc))
    finally:
        if stream is not None:stream.release()
        (out/'review.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=out.name,status=r['status'],error=r.get('error'),frames=len(r['frames']),motor_api_used=False),ensure_ascii=False))
if __name__=='__main__':main()
