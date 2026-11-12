"""Operator camera-only calibration view: red ID2 + green ID5 + wrist ID41."""
import argparse,datetime as dt,functools,json,multiprocessing as mp,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def worker(send,stop,preview=None,*,folder,start):
    directory=Path(folder);cap=session=cv=None
    result=dict(status='started',profiles=[],motors_accessed=False,motor_port_opened=False,motor_commands_sent=0,
        requested_scene='cube ON TABLE, RED UP, GREEN toward camera; wrist ID41 also visible; hand unchanged',
        marker40_required=False,space_is_not_marker_gated=True,geometry_registration_verified=False)
    try:
        import cv2 as cv
        import numpy as np
        from camera_stream import enumerate_names,select_camera,read_frame
        from camera_720_check import open_hd
        from camera_exposure_sweep import ExposureSession
        from marker_detector import MarkerDetector
        names=enumerate_names();send.send(('devices',names));index=select_camera(names);send.send(('selected',dict(camera_index=index,device_name=names[index])))
        cap,meta,attempts=open_hd(cv,index,require_mjpg=True,backends=((cv.CAP_DSHOW,'DSHOW'),));session=ExposureSession(cap,cv.CAP_PROP_AUTO_EXPOSURE,cv.CAP_PROP_EXPOSURE)
        result.update(camera=dict(meta,device_name=names[index],attempts=attempts),original_camera_settings=session.original)
        title=preview['title'];cv.namedWindow(title,cv.WINDOW_NORMAL);cv.resizeWindow(title,1100,650)
        detector=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),[2,5],hand_id=41,required_cube_count=1);send.send(('ready',dict(meta,camera_index=index,device_name=names[index])))
        def packet(label,name=None):
            if stop.is_set():raise KeyboardInterrupt('User stop')
            item=read_frame(cap,stop)
            if item is None:raise KeyboardInterrupt('User stop')
            stamp,frame=item;send.send(('frame',item));gray=cv.cvtColor(frame,cv.COLOR_BGR2GRAY);corners,ids,_=detector.detectMarkers(gray);decoded=[] if ids is None else ids.ravel().tolist()
            q=dict(ids=decoded,camera_read_monotonic=float(stamp),corners_px_by_id={str(i):np.asarray(c).reshape(4,2).tolist() for i,c in zip(decoded,corners)},detector=dict(detector.stats),
                red_and_green_visible=2 in decoded and 5 in decoded,red_green_and_anchor_visible=all(i in decoded for i in (2,5,41)),white_clipped_fraction=float(np.mean(np.all(frame>=250,axis=2))))
            show=frame.copy();detector.draw(show)
            lines=[label,'CAMERA ONLY. MOTOR POWER OFF. CUBE ON TABLE; HAND STAYS UNCHANGED.',
                'RED UP (ID2), GREEN toward CAMERA (ID5), wrist ID41 in SAME view.',
                'Avoid glare on RED marker black border. SPACE saves even if a marker is missing. Q exits.']
            for n,line in enumerate(lines):cv.putText(show,line,(10,28+n*27),cv.FONT_HERSHEY_SIMPLEX,.52,(0,255,255),2)
            cv.imshow(title,show);key=cv.waitKey(1)&255
            if key in (ord('q'),ord('Q'),27) or cv.getWindowProperty(title,cv.WND_PROP_VISIBLE)<1:raise KeyboardInterrupt('Camera user stop')
            if key==32:start.set()
            if name is not None:
                ok,png=cv.imencode('.png',frame)
                if not ok:raise RuntimeError('PNG encoding failed')
                (directory/name).write_bytes(png.tobytes());q['image']=name
            return q
        while not start.is_set():packet('RED + GREEN reference: SPACE once starts capture')
        result['started_monotonic']=time.monotonic()
        for label,value in (('original',None),('manual_minus5',-5.),('manual_minus6',-6.)):
            if time.monotonic()-result['started_monotonic']>30:raise RuntimeError('Capture exceeded 30s; partial images retained')
            settings=dict(supported=True,readback=session.original) if value is None else session.apply(value)
            if value is not None and not settings['supported']:result['profiles'].append(dict(name=label,settings=settings,skipped=True,frames=[]));continue
            began=time.monotonic()
            while time.monotonic()-began<.5:packet('Settling exposure; hold cube and camera still')
            frames=[packet(f'Capturing {label}; hold still',f'{label}_{i:02}.png') for i in range(6)]
            result['profiles'].append(dict(name=label,settings=settings,frames=frames));print('Saved',label,len(frames),'frames; red/green/anchor',sum(v['red_green_and_anchor_visible'] for v in frames),'/6',flush=True)
        result.update(status='cube_red_green_capture_complete',saved_frames=sum(len(v['frames']) for v in result['profiles']),do_not_repeat_without_saved_file_review=True)
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
    if not args.capture:print('CHECK ONLY: no camera, motor API, port or settings opened. --capture opens CAMERA ONLY.');return 0
    folder=ROOT/('cube_red_green_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir();report=dict(status='started',motors_accessed=False,motor_port_opened=False,motor_commands_sent=0);camera=None
    try:
        import msvcrt
        from camera_stream import CameraStream
        print('CAMERA ONLY. Motor power OFF. Leave hand, base and book unchanged.',flush=True)
        print('Cube on TABLE OUTSIDE hand: RED UP, GREEN toward camera. Show red ID2 + green ID5 + wrist ID41 together.',flush=True)
        print('Do not use ORANGE as the front when RED is up: orange is the opposite face underneath.',flush=True)
        print('SPACE once in CAMERA or TERMINAL captures about10s. Missing markers never block saving. Q exits.',flush=True)
        context=mp.get_context('spawn');start=context.Event();camera=CameraStream(report,worker=functools.partial(worker,folder=str(folder),start=start),preview_title='RED GREEN - CAMERA ONLY');began=None
        while not (folder/'worker_result.json').is_file():
            while msvcrt.kbhit():
                key=msvcrt.getwch()
                if key in ('q','Q','\x1b','\x03'):raise KeyboardInterrupt('User stop')
                if key==' ':start.set()
            if start.is_set():
                if began is None:began=time.monotonic()
                if time.monotonic()-began>40:raise RuntimeError('Camera capture exceeded 40s')
            camera.read_packet();time.sleep(.02)
        report.update(json.loads((folder/'worker_result.json').read_bytes()))
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc) or 'Interrupted')
    finally:
        if camera is not None:camera.release()
        payload=json.dumps(report,indent=2)+'\n';(folder/'result.json').write_text(payload,encoding='utf-8');(ROOT/'leap_cube_red_green_result.json').write_text(payload,encoding='utf-8')
    print('Saved:',folder,flush=True);print('Capture finished. Do not repeat; say done. No motor commands.',flush=True)
    return 0 if report.get('status')=='cube_red_green_capture_complete' else 1
if __name__=='__main__':raise SystemExit(main())
