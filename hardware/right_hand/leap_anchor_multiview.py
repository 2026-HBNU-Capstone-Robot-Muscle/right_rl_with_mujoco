"""Three camera-only views of the fixed ID40/41 pair. No motor imports."""
import argparse
import datetime as dt
import json
import time
from pathlib import Path
import numpy as np
import cv2 as cv
from marker_detector import MarkerDetector
from camera_stream import CameraStream

ROOT=Path(__file__).resolve().parent
SIDES={40:.0282,41:.027}

def square(side):
    s=side/2
    return np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]],np.float64)

def pair_observation(corners,ids,K,dist):
    ids=[] if ids is None else [int(i) for i in np.asarray(ids).ravel()]
    if ids.count(40)!=1 or ids.count(41)!=1:return None
    row={}
    for ident,side in SIDES.items():
        pts=np.asarray(corners[ids.index(ident)],np.float64).reshape(4,2)
        ok,rv,tv=cv.solvePnP(square(side),pts,K,dist)
        if not ok or tv[2,0]<=0 or not np.isfinite(rv).all() or not np.isfinite(tv).all():return None
        predicted=cv.projectPoints(square(side),rv,tv,K,dist)[0].reshape(4,2)
        error=float(np.linalg.norm(predicted-pts,axis=1).mean())
        row[str(ident)]=dict(corners_px=pts.tolist(),rvec=rv.ravel().tolist(),
                            tvec_m=tv.ravel().tolist(),reprojection_px=error)
    return row

def poll_key():
    key=cv.waitKey(1)&255
    if key in (ord('q'),ord('Q'),27,3):return key
    import msvcrt
    while msvcrt.kbhit():
        char=msvcrt.getwch()
        if char in ('\x00','\xe0'):
            if msvcrt.kbhit():msvcrt.getwch()
            continue
        if char in ('q','Q','\x1b','\x03'):return ord('q')
        if char==' ':key=32
    return key

def main(capture=False):
    with np.load(ROOT/'camera1_720.npz') as z:
        K,dist=z['K'].copy(),z['dist'].copy()
    if not capture:
        print('CHECK ONLY: 3 views x 5 frames, ID40 28.2mm, ID41 approximately 27mm. No devices opened.')
        return 0
    folder=ROOT/('anchor_multiview_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir()
    report=dict(status='incomplete',motor_port_opened=False,policy_actions_sent=0,
                folder=str(folder),intrinsic=K.tolist(),distortion=dist.ravel().tolist(),
                marker_sides_m={str(k):v for k,v in SIDES.items()},frames=[],
                dimension_note='ID41 is an approximate user measurement; not a metric accuracy certificate.')
    def save():
        data=json.dumps(report,ensure_ascii=False,indent=2)
        (folder/'result.json').write_text(data,encoding='utf-8')
        (ROOT/'leap_anchor_multiview_result.json').write_text(data,encoding='utf-8')
    camera=None;view=0;burst=None;last_frame=time.monotonic();last_save=-float('inf')
    labels=['View 1: normal front view','View 2: move CAMERA to one side (about 15-25 degrees)',
            'View 3: move CAMERA to the other side (about 15-25 degrees)']
    try:
        print('CAMERA ONLY. Motor power OFF, cube removed. Keep base and both markers fixed.',flush=True)
        print('Move only the CAMERA, not the hand or markers. Do not zoom/refocus/change resolution.',flush=True)
        print('Show BOTH ID40 and ID41. SPACE in CAMERA or TERMINAL saves each view. Q exits.',flush=True)
        camera=CameraStream(report)
        detector=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),
                                [41],hand_id=40,required_cube_count=1)
        title='ANCHOR CALIBRATION | NO MOTOR PORT | 3 VIEWS'
        cv.namedWindow(title,cv.WINDOW_NORMAL);cv.resizeWindow(title,1100,620)
        print(labels[view],flush=True)
        current=None
        while view<3:
            packet=camera.read_packet()
            if packet is not None:
                stamp,raw=packet;last_frame=time.monotonic()
                if time.monotonic()-stamp>.3:continue
                corners,ids,_=detector.detectMarkers(cv.cvtColor(raw,cv.COLOR_BGR2GRAY))
                pair=pair_observation(corners,ids,K,dist)
                current=(stamp,raw,pair)
                if burst is not None and pair is not None and stamp>burst and stamp-last_save>=.12:
                    count=sum(r['view']==view for r in report['frames'])
                    path=folder/f'view_{view+1}_{count+1}.png'
                    ok,png=cv.imencode('.png',raw)
                    if not ok:raise RuntimeError('PNG encoding failed')
                    path.write_bytes(png.tobytes())
                    report['frames'].append(dict(view=view,captured_at_unix_s=time.time()-(time.monotonic()-stamp),
                                                 image=path.name,markers=pair))
                    last_save=stamp;save()
                    if count+1==5:
                        print(f'Saved view {view+1}/3.',flush=True)
                        view+=1;burst=None;current=None
                        if view==3:break
                        print(labels[view],flush=True)
                display=raw.copy()
                found=[] if ids is None else list(map(int,ids.ravel()))
                angle_text=''
                if pair is not None and report['frames']:
                    first=report['frames'][0]['markers']['41']['rvec']
                    old_normal=cv.Rodrigues(np.asarray(first))[0][:,2]
                    new_normal=cv.Rodrigues(np.asarray(pair['41']['rvec']))[0][:,2]
                    angle=np.degrees(np.arccos(np.clip(old_normal@new_normal,-1,1)))
                    angle_text=f' | angle from first view: {angle:.1f} deg'
                lines=['MOTOR POWER OFF | BASE/MARKERS FIXED | CUBE REMOVED',labels[view],
                       f'IDs {found} | '+('PAIR READY' if pair else 'Show BOTH 40 and 41')+angle_text,
                       'Hold CAMERA still: capturing 5 fresh frames' if burst is not None else 'SPACE saves this view; Q exits']
                for line,text in enumerate(lines):
                    cv.putText(display,text,(10,30+line*28),cv.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),2)
                cv.imshow(title,display)
            if time.monotonic()-last_frame>3:raise RuntimeError('No fresh camera frame for 3 seconds')
            key=poll_key()
            if key in (ord('q'),ord('Q'),27,3):raise KeyboardInterrupt('User stop')
            if key==32 and burst is None:
                if current is None or time.monotonic()-current[0]>.3 or current[2] is None:
                    print('Not saved yet: show BOTH ID40 and ID41, then SPACE.',flush=True)
                else:burst=time.monotonic()
            time.sleep(.005)
        report['status']='three_views_captured_unvalidated'
        print('Three views saved. OFFLINE calibration still required; no hand motion occurred.',flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc));print('STOP:',exc,flush=True)
    finally:
        try:
            if camera is not None:camera.release()
            cv.destroyAllWindows()
        except Exception as exc:
            report.update(status='stopped',close_error=str(exc))
        finally:
            save();print('Saved:',folder,flush=True)
    return 0 if report['status']=='three_views_captured_unvalidated' else 1

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--capture',action='store_true')
    raise SystemExit(main(p.parse_args().capture))
