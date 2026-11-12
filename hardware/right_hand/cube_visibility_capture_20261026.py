"""Operator-started camera-only cube/ID41 visibility capture. No motor API."""
import argparse,datetime as dt,functools,json,multiprocessing as mp,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def quality(ids,edges,clipped,sharpness=None):
    ids=list(map(int,ids));minimum=float(edges.get('5',0.));pair=5 in ids and 41 in ids
    return dict(ids=ids,minimum_edge_px_by_id=dict(edges),green_short_edge_px=minimum,anchor_and_green_visible=pair,
        advisory_ready=bool(pair and minimum>=40),white_clipped_fraction=float(clipped),green_roi_sharpness=sharpness)

def worker(send,stop,preview=None,*,folder,start):
    cap=session=cv=None;directory=Path(folder)
    result=dict(status='started',profiles=[],motors_accessed=False,motor_port_opened=False,motor_commands_sent=0,
        geometry_registration_verified=False,runtime_configuration_installed=False,space_is_not_marker_gated=True,
        requested_cube_reference='GREEN UP on the table, beside wrist ID41; camera looks down toward its top',table_level_assumed=True)
    try:
        import cv2 as cv
        import numpy as np
        from camera_stream import enumerate_names,select_camera,read_frame
        from camera_720_check import open_hd
        from camera_exposure_sweep import ExposureSession
        from marker_detector import MarkerDetector
        names=enumerate_names();send.send(('devices',names));index=select_camera(names)
        send.send(('selected',dict(camera_index=index,device_name=names[index])))
        cap,meta,attempts=open_hd(cv,index,require_mjpg=True,backends=((cv.CAP_DSHOW,'DSHOW'),))
        session=ExposureSession(cap,cv.CAP_PROP_AUTO_EXPOSURE,cv.CAP_PROP_EXPOSURE)
        result.update(camera=dict(meta,device_name=names[index],attempts=attempts),original_settings=session.original,
            focus_readbacks=dict(autofocus=float(cap.get(cv.CAP_PROP_AUTOFOCUS)),focus=float(cap.get(cv.CAP_PROP_FOCUS)),zoom=float(cap.get(cv.CAP_PROP_ZOOM))))
        title=preview['title'];cv.namedWindow(title,cv.WINDOW_NORMAL);cv.resizeWindow(title,1100,650)
        detector=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),[5],hand_id=41,required_cube_count=1)
        send.send(('ready',dict(meta,camera_index=index,device_name=names[index])))
        def packet(label,save=None):
            if stop.is_set():raise KeyboardInterrupt('User stop')
            item=read_frame(cap,stop)
            if item is None:raise KeyboardInterrupt('User stop')
            stamp,frame=item;send.send(('frame',item));gray=cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
            corners,ids,_=detector.detectMarkers(gray);decoded=[] if ids is None else ids.ravel().tolist();sharp=None
            if 5 in decoded:
                pts=np.asarray(corners[decoded.index(5)]).reshape(4,2);x0,y0=np.floor(pts.min(0)-8).astype(int);x1,y1=np.ceil(pts.max(0)+8).astype(int)
                roi=gray[max(0,y0):min(gray.shape[0],y1),max(0,x0):min(gray.shape[1],x1)]
                if roi.size:sharp=float(cv.Laplacian(roi,cv.CV_64F).var())
            q=quality(decoded,detector.stats['min_edge_px_by_id'],np.mean(np.all(frame>=250,axis=2)),sharp)
            q.update(camera_read_monotonic=float(stamp),corners_px_by_id={str(mid):np.asarray(co).reshape(4,2).tolist() for mid,co in zip(decoded,corners)},detector=dict(detector.stats))
            show=frame.copy();detector.draw(show)
            messages=[label,'CUBE ON TABLE BESIDE WRIST ID41. GREEN UP. CAMERA LOOKS DOWN AT ID5. MOTOR POWER OFF.',
                f"ID5 short edge {q['green_short_edge_px']:.0f}px (aim >=40). ID41 visible: {41 in decoded}. No ID40 requirement.",
                'Aim more from ABOVE and closer, keeping ID41 visible. SPACE saves even weak markers. Q exits.']
            for n,line in enumerate(messages):cv.putText(show,line,(10,28+n*27),cv.FONT_HERSHEY_SIMPLEX,.52,(0,255,255),2)
            cv.imshow(title,show);key=cv.waitKey(1)&255
            if key in (ord('q'),ord('Q'),27) or cv.getWindowProperty(title,cv.WND_PROP_VISIBLE)<1:raise KeyboardInterrupt('Camera user stop')
            if key==32:start.set()
            if save is not None:
                ok,png=cv.imencode('.png',frame)
                if not ok:raise RuntimeError('PNG encoding failed')
                (directory/save).write_bytes(png.tobytes());q['image']=save
            return q
        while not start.is_set():packet('CAMERA ONLY: show GREEN ID5 + wrist ID41; SPACE starts capture')
        result['started_monotonic']=time.monotonic()
        for label,value in (('original',None),('manual_minus5',-5.),('manual_minus6',-6.),('manual_minus7',-7.)):
            if time.monotonic()-result['started_monotonic']>35:raise RuntimeError('Capture exceeded 35s; partial images retained')
            settings=dict(supported=True,readback=session.original) if value is None else session.apply(value)
            if value is not None and not settings['supported']:
                result['profiles'].append(dict(name=label,settings=settings,skipped=True,frames=[]));continue
            settled=time.monotonic()
            while time.monotonic()-settled<.5:packet('Settling exposure; hold cube and camera still')
            frames=[packet(f'Capturing {label}; hold still',f'{label}_{i:02}.png') for i in range(6)]
            result['profiles'].append(dict(name=label,settings=settings,frames=frames,readable_frames=sum(v['advisory_ready'] for v in frames)))
            print('CAPTURE',label,'readable GREEN+ID41',sum(v['advisory_ready'] for v in frames),'/6',flush=True)
        rows=[v for profile in result['profiles'] for v in profile.get('frames',[])];readable=sum(v['advisory_ready'] for v in rows)
        result.update(status='cube_visibility_capture_complete',saved_frames=len(rows),readable_frames=readable,
            readable_pair_observed=readable>=3,do_not_repeat_without_saved_file_review=True)
    except (Exception,KeyboardInterrupt) as exc:
        result.update(status='stopped',error=str(exc) or 'Interrupted')
        try:send.send(('error',result['error']))
        except (BrokenPipeError,EOFError,OSError):pass
    finally:
        if session is not None:
            try:result['camera_restoration']=session.restore()
            except Exception as exc:result['camera_restoration']=dict(restored_original_readbacks=False,error=str(exc))
        (directory/'worker_result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
        try:
            if cap is not None:cap.release()
            if cv is not None:cv.destroyAllWindows()
        finally:stop.wait(2);send.close()

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--capture',action='store_true');args=parser.parse_args(argv)
    if not args.capture:
        print('CHECK ONLY: no camera, serial port, or settings opened. --capture opens CAMERA ONLY.');return 0
    folder=ROOT/('cube_visibility_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir();report=dict(status='started',motors_accessed=False,motor_port_opened=False,motor_commands_sent=0)
    camera=None
    try:
        import msvcrt
        from camera_stream import CameraStream
        print('CAMERA ONLY. Motor power OFF. Cube on TABLE beside wrist ID41, OUTSIDE the robot hand.',flush=True)
        print('GREEN face UP on the table. Aim camera more from ABOVE at center ID5. Keep wrist ID41 in the SAME picture. No ID40 requirement.',flush=True)
        print('SPACE once in CAMERA or TERMINAL starts about15s capture. Even weak marker images are saved. Q exits.',flush=True)
        context=mp.get_context('spawn');start=context.Event();camera=CameraStream(report,worker=functools.partial(worker,folder=str(folder),start=start),preview_title='CUBE VISIBILITY - CAMERA ONLY')
        began=None
        while not (folder/'worker_result.json').is_file():
            while msvcrt.kbhit():
                key=msvcrt.getwch()
                if key in ('q','Q','\x1b','\x03'):raise KeyboardInterrupt('User stop')
                if key==' ':start.set()
            if start.is_set():
                if began is None:began=time.monotonic()
                if time.monotonic()-began>45:raise RuntimeError('Camera capture exceeded 45s')
            camera.read_packet();time.sleep(.02)
        report.update(json.loads((folder/'worker_result.json').read_bytes()))
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc) or 'Interrupted')
    finally:
        if camera is not None:camera.release()
        payload=json.dumps(report,indent=2)+'\n';(folder/'result.json').write_text(payload,encoding='utf-8');(ROOT/'leap_cube_visibility_result.json').write_text(payload,encoding='utf-8')
    print('Saved:',folder,flush=True)
    print('Capture complete. Do not repeat; say done for saved-file review. No motor port or motor commands.',flush=True)
    return 0 if report.get('status')=='cube_visibility_capture_complete' else 1
if __name__=='__main__':raise SystemExit(main())
