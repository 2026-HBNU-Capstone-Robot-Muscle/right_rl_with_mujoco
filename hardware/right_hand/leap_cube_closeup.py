"""One camera-only close-up burst; no robot imports, no pose certification."""
import argparse
import datetime as dt
import json
from pathlib import Path
import time
import cv2 as cv
import numpy as np
from camera_stream import CameraStream
from marker_detector import MarkerDetector
from leap_anchor_multiview import poll_key

ROOT=Path(__file__).resolve().parent

def quality(corners,ids,shape):
    decoded=[] if ids is None else [int(i) for i in np.asarray(ids).ravel()]
    if decoded.count(5)!=1:return dict(ready=False,reason='Show green center ID5')
    pts=np.asarray(corners[decoded.index(5)]).reshape(4,2)
    if not np.isfinite(pts).all():return dict(ready=False,reason='Invalid corners')
    edges=np.linalg.norm(pts-np.roll(pts,1,axis=0),axis=1)
    if min(edges)<80:return dict(ready=False,reason=f'Move camera closer: ID5 short edge {min(edges):.0f}px / need 80px')
    if max(edges)/min(edges)>1.3:return dict(ready=False,reason='Aim more squarely at the GREEN face')
    # Framing hint only. This is not a calibrated face detection.
    face=pts.mean(0)+(pts-pts.mean(0))*(55/10.7)*1.08
    h,w=shape[:2]
    if np.any(face[:,0]<15) or np.any(face[:,0]>w-15) or np.any(face[:,1]<15) or np.any(face[:,1]>h-15):
        return dict(ready=False,reason='Center the WHOLE green face with space around its white rim')
    return dict(ready=True,reason='READY - SPACE saves 10 fresh frames',corners_px=pts.tolist(),min_edge_px=float(min(edges)))

def main(capture=False):
    if not capture:
        print('CHECK ONLY. Camera-only ID5 close-up, one SPACE, 10 fresh frames. No devices opened.');return 0
    folder=ROOT/('cube_closeup_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report=dict(status='incomplete',motor_port_opened=False,policy_actions_sent=0,folder=str(folder),frames=[],
                measurements=json.loads((ROOT/'cube_measurement_20261014.json').read_text()))
    camera=None;burst=None;last_saved=-float('inf');last_fresh=time.monotonic();current=None
    def save():
        payload=json.dumps(report,ensure_ascii=False,indent=2)+'\n'
        (folder/'result.json').write_text(payload,encoding='utf-8')
        (ROOT/'leap_cube_closeup_result.json').write_text(payload,encoding='utf-8')
    try:
        print('CAMERA ONLY. Motor power OFF. Put cube on table OUTSIDE the robot hand.',flush=True)
        print('Face GREEN center ID5 toward camera. Show entire white face rim; avoid glare.',flush=True)
        print('Move camera closer until READY. SPACE in CAMERA or TERMINAL once; hold still. Q exits.',flush=True)
        camera=CameraStream(report)
        detector=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),[5],hand_id=5,required_cube_count=1)
        title='CUBE CLOSE-UP | NO MOTOR PORT'
        cv.namedWindow(title,cv.WINDOW_NORMAL);cv.resizeWindow(title,1100,620)
        while len(report['frames'])<10:
            packet=camera.read_packet()
            if packet is not None:
                stamp,raw=packet
                if time.monotonic()-stamp>.3:continue
                last_fresh=time.monotonic()
                corners,ids,_=detector.detectMarkers(cv.cvtColor(raw,cv.COLOR_BGR2GRAY))
                check=quality(corners,ids,raw.shape);current=(stamp,check)
                if burst is not None and check['ready'] and stamp>burst and stamp-last_saved>=.12:
                    name=f"frame_{len(report['frames'])+1:02}.png";ok,png=cv.imencode('.png',raw)
                    if not ok:raise RuntimeError('PNG encoding failed')
                    (folder/name).write_bytes(png.tobytes())
                    report['frames'].append(dict(image=name,captured_at_unix_s=time.time()-(time.monotonic()-stamp),quality=check,detector=dict(detector.stats)))
                    last_saved=stamp;save()
                display=raw.copy();detector.draw(display)
                for i,line in enumerate(['POWER OFF | CUBE ON TABLE | GREEN FACE TOWARD CAMERA',check['reason'],f"Saving {len(report['frames'])}/10 - hold still" if burst is not None else 'SPACE once when READY; Q exits']):
                    cv.putText(display,line,(10,30+30*i),cv.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),2)
                cv.imshow(title,display)
            if time.monotonic()-last_fresh>3:raise RuntimeError('No fresh camera frame for 3s')
            key=poll_key()
            if key in (ord('q'),ord('Q'),27,3):raise KeyboardInterrupt('User stop')
            if key==32 and burst is None:
                if current is None or time.monotonic()-current[0]>.3 or not current[1]['ready']:
                    print('Not recording: '+(current[1]['reason'] if current else 'waiting for frame'),flush=True)
                else:burst=time.monotonic();print('Recording 10 close-up frames.',flush=True)
            if burst is not None and time.monotonic()-burst>15:raise RuntimeError('Close-up quality lost; partial images saved')
            time.sleep(.005)
        report['status']='closeup_captured_unvalidated';print('Close-up saved. No motor motion or policy action.',flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc));print('STOP:',exc,flush=True)
    finally:
        try:
            if camera is not None:camera.release()
            cv.destroyAllWindows()
        finally:save();print('Saved:',folder,flush=True)
    return 0 if report['status']=='closeup_captured_unvalidated' else 1

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--capture',action='store_true')
    raise SystemExit(main(parser.parse_args().capture))
