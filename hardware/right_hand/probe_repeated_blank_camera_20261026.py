"""Bounded camera-only signal probe. No motor imports or APIs."""
import datetime,json,time
from pathlib import Path
import cv2
from camera_stream import CameraStream
from camera_frame_signal import frame_signal

def main():
    folder=Path(__file__).resolve().parent/('blank_camera_probe_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    r=dict(motor_port_opened=False,motor_commands_sent=0,frames=[],status='not_started');cam=None;start=time.monotonic()
    try:
        cam=CameraStream(r)
        end=time.monotonic()+40;last_saved=0.
        while time.monotonic()<end:
            packet=cam.read_packet()
            if packet is None:time.sleep(.005);continue
            stamp,frame=packet;q=frame_signal(frame)
            r['frames'].append(dict(stamp=stamp,**q))
            if (not q['usable'] or stamp-last_saved>10):
                name='frame_'+str(len(r['frames'])).zfill(5)+'.jpg';ok,jpg=cv2.imencode('.jpg',frame)
                if ok:(folder/name).write_bytes(jpg.tobytes())
                last_saved=stamp
        r.update(status='observed',usable=sum(f['usable'] for f in r['frames']),blank=sum(not f['usable'] for f in r['frames']))
    except Exception as exc:r.update(status='failed',error=str(exc))
    finally:
        if cam is not None:cam.release()
        r['elapsed_s']=time.monotonic()-start;(folder/'result.json').write_text(json.dumps(r,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in r.items() if k!='frames'},ensure_ascii=False));print('Saved',folder)
if __name__=='__main__':main()
