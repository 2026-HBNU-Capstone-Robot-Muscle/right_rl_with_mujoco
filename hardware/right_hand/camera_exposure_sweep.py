"""User-triggered CAMERA ONLY exposure diagnosis. No motor APIs or 3D pose use."""
import argparse
import datetime as dt
import functools
import json
import math
import multiprocessing as mp
from pathlib import Path
import shutil
import time

ROOT=Path(__file__).resolve().parent


def exposure_values(current):
    # DirectShow commonly exposes negative log exposure values. Unknown/non-
    # negative getters are not treated as a known range or a supported control.
    if not math.isfinite(current) or not -13<=current<=-1:
        return [-3.,-4.,-5.,-6.,-7.]
    first=float(round(current))
    return list(dict.fromkeys([first,max(-13.,first-1),max(-13.,first-2),
                               max(-13.,first-3),max(-13.,first-4)]))


class ExposureSession:
    """Best-effort reversible camera property probing with explicit readbacks."""
    def __init__(self,cap,auto_property,exposure_property):
        self.cap=cap;self.auto_property=auto_property;self.exposure_property=exposure_property
        self.original=dict(auto=float(cap.get(auto_property)),exposure=float(cap.get(exposure_property)))
        self.attempted=False;self.restoration=None

    def apply(self,value):
        if not all(math.isfinite(v) for v in self.original.values()):
            return dict(supported=False,error='Original camera settings unavailable; no property changes attempted')
        self.attempted=True
        automatic=bool(self.cap.set(self.auto_property,.25))
        manual=bool(self.cap.set(self.exposure_property,float(value))) if automatic else False
        actual=dict(auto=float(self.cap.get(self.auto_property)),exposure=float(self.cap.get(self.exposure_property)))
        matches=math.isfinite(actual['exposure']) and abs(actual['exposure']-value)<=.25
        return dict(requested_exposure=value,set_auto_returned=automatic,set_exposure_returned=manual,
                    readback=actual,supported=bool(automatic and manual and matches))

    def restore(self):
        if not self.attempted:
            readable=all(math.isfinite(v) for v in self.original.values())
            self.restoration=dict(changes_attempted=False,restore_not_needed=True,
                                  restored_original_readbacks=readable)
            return self.restoration
        errors=[];returns={}
        # Exposure first, automatic mode last, so the original mode wins.
        for name,prop in (('exposure',self.exposure_property),('auto',self.auto_property)):
            try:returns[name]=bool(self.cap.set(prop,self.original[name]))
            except Exception as exc:errors.append(str(exc))
        readback={}
        for name,prop in (('auto',self.auto_property),('exposure',self.exposure_property)):
            try:readback[name]=float(self.cap.get(prop))
            except Exception as exc:errors.append(str(exc))
        exact=all(math.isfinite(readback.get(k,float('nan'))) and abs(readback[k]-v)<=.25
                  for k,v in self.original.items())
        self.restoration=dict(changes_attempted=True,set_returns=returns,readback=readback,
            restored_original_readbacks=bool(not errors and all(returns.values()) and exact),errors=errors,
            note='Camera auto mode can change its exposure readback again. Readbacks are recorded; restoration is not claimed when they do not match.')
        return self.restoration


def frame_quality(ids,edge_sizes,clipped_fraction):
    ids=list(map(int,ids));edges={str(k):float(v) for k,v in edge_sizes.items()}
    pair=ids.count(40)==1 and ids.count(5)==1
    shortest=min(edges.get('40',0.),edges.get('5',0.)) if pair else 0.
    return dict(ids=ids,minimum_edge_px=edges,paired_markers=pair,
                usable_pair=bool(pair and shortest>=24),shortest_pair_edge_px=shortest,
                clipped_white_fraction=float(clipped_fraction))


def quality_rank(row):
    return (bool(row['usable_pair']),bool(row['paired_markers']),40 in row['ids'],5 in row['ids'],
            row['shortest_pair_edge_px'],-row['clipped_white_fraction'])


def sweep_worker(send,stop,preview=None,*,folder,start):
    """All camera calls and window handling stay in a disposable process."""
    cap=None;session=None;cv=None;directory=Path(folder)
    result=dict(status='started',profiles=[],motor_commands_sent=0,motors_accessed=False,
                runtime_settings_installed=False,registration_verified=False)
    try:
        import cv2 as cv
        import numpy as np
        from camera_stream import enumerate_names,select_camera,read_frame
        from camera_720_check import open_hd
        from marker_detector import MarkerDetector
        names=enumerate_names();send.send(('devices',names));index=select_camera(names)
        send.send(('selected',dict(camera_index=index,device_name=names[index])))
        cap,meta,attempts=open_hd(cv,index,require_mjpg=True,backends=((cv.CAP_DSHOW,'DSHOW'),))
        session=ExposureSession(cap,cv.CAP_PROP_AUTO_EXPOSURE,cv.CAP_PROP_EXPOSURE)
        result.update(camera=dict(meta,device_name=names[index],camera_index=index),original_settings=session.original)
        title=preview['title'];cv.namedWindow(title,cv.WINDOW_NORMAL);cv.resizeWindow(title,1000,600)
        detector=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),range(6),hand_id=40,required_cube_count=1)
        send.send(('ready',dict(meta,camera_index=index,device_name=names[index])))

        def packet(label,save=None):
            if stop.is_set():raise KeyboardInterrupt('User stop')
            item=read_frame(cap,stop)
            if item is None:raise KeyboardInterrupt('User stop')
            stamp,frame=item;send.send(('frame',item))
            gray=cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
            _,ids,_=detector.detectMarkers(gray)
            found=[] if ids is None else ids.ravel().tolist()
            clipped=float(np.mean(np.all(frame>=250,axis=2)))
            quality=frame_quality(found,detector.stats['min_edge_px_by_id'],clipped)
            show=frame.copy();detector.draw(show)
            cv.putText(show,label,(10,30),cv.FONT_HERSHEY_SIMPLEX,.65,(0,255,255),2)
            cv.imshow(title,show);key=cv.waitKey(1)&255
            if key in (ord('q'),ord('Q'),27) or cv.getWindowProperty(title,cv.WND_PROP_VISIBLE)<1:
                raise KeyboardInterrupt('Camera user stop')
            if key==32:start.set()
            if save is not None:
                target=directory/save
                if not cv.imwrite(str(target.name),frame):raise RuntimeError('Cannot save raw frame')
                quality.update(image=save,camera_read_monotonic=stamp)
            return quality

        # Work in the Unicode folder with ASCII relative filenames for codecs.
        import os
        os.chdir(directory)
        while not start.is_set():packet('CAMERA ONLY: palm ID40 + cube ID5 beside hand; SPACE starts; Q exits')
        profiles=[('original',None)]+[(f'manual_{n}',value) for n,value in enumerate(exposure_values(session.original['exposure']))]
        result['sweep_started_monotonic']=time.monotonic()
        all_rows=[]
        for name,value in profiles:
            if time.monotonic()-result['sweep_started_monotonic']>35:
                raise RuntimeError('Camera sweep exceeded 35s')
            applied=dict(supported=True,readback=session.original) if value is None else session.apply(value)
            if value is not None and not applied['supported']:
                result['profiles'].append(dict(name=name,settings=applied,frames=[],skipped=True))
                # Unsupported setters/readbacks are not treated as a working configuration.
                continue
            for _ in range(6):packet('CAMERA ONLY: setting exposure; hold camera/hand/cube still')
            rows=[]
            for n in range(6):
                row=packet(f'CAMERA ONLY: recording {name}; hold still; Q exits',f'{name}_{n:02d}.jpg')
                rows.append(row);all_rows.append(row)
            best=max(rows,key=quality_rank)
            after=dict(auto=float(cap.get(cv.CAP_PROP_AUTO_EXPOSURE)),
                       exposure=float(cap.get(cv.CAP_PROP_EXPOSURE)))
            result['profiles'].append(dict(name=name,settings=applied,frames=rows,
                settings_after_capture=after,
                usable_pair_frames=sum(r['usable_pair'] for r in rows),best_image=best['image']))
            print('EXPOSURE',name,'readable palm+cube',sum(r['usable_pair'] for r in rows),'/6',flush=True)
        measured=[p for p in result['profiles'] if p.get('frames')]
        best_profile=max(measured,key=lambda p:(p['usable_pair_frames'],
            sum(r['paired_markers'] for r in p['frames']),max(quality_rank(r) for r in p['frames'])))
        best=max(best_profile['frames'],key=quality_rank)
        shutil.copyfile(directory/best['image'],directory/'best_frame.jpg')
        result.update(status='camera_exposure_diagnosis_complete',best_frame=best,
            best_image='best_frame.jpg',best_profile=best_profile['name'],
            usable_pair_observed=any(p['usable_pair_frames']>=3 for p in measured),
            usable_pair_minimum_frames=3,
            saved_raw_frames=len(all_rows))
    except (Exception,KeyboardInterrupt) as exc:
        result.update(status='stopped',error=str(exc) or 'Interrupted')
        try:send.send(('error',result['error']))
        except (BrokenPipeError,EOFError,OSError):pass
    finally:
        if session is not None:
            try:result['restoration']=session.restore()
            except Exception as exc:result['restoration']=dict(restored_original_readbacks=False,error=str(exc))
        # Save metadata before potentially blocking native cleanup.
        pending=directory/'worker_result.pending'
        pending.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        pending.replace(directory/'worker_result.json')
        try:
            if cap is not None:cap.release()
            if cv is not None:cv.destroyAllWindows()
        finally:
            stop.wait(2)
            send.close()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--execute',action='store_true')
    args=parser.parse_args(argv)
    print('CAMERA ONLY EXPOSURE CHECK. Motor power OFF; no motor API, target or torque changes.',flush=True)
    if not args.execute:
        print('CHECK ONLY: no camera, serial port, or settings opened.');return 0
    folder=ROOT/('camera_exposure_sweep_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report=dict(timestamp=dt.datetime.now().astimezone().isoformat(),status='started',motors_accessed=False)
    camera=None
    try:
        from camera_stream import CameraStream
        import msvcrt
        context=mp.get_context('spawn');start=context.Event()
        worker=functools.partial(sweep_worker,folder=str(folder),start=start)
        print('Cube on table OUTSIDE hand. Show palm ID40 + green center ID5 together. ID41 not required.',flush=True)
        print('SPACE in TERMINAL or CAMERA starts automatic capture; hold still for about 15-30s. Q exits.',flush=True)
        camera=CameraStream(report,worker=worker,preview_title='LEAP camera exposure check')
        started_at=None
        while not (folder/'worker_result.json').is_file():
            if msvcrt.kbhit():
                key=msvcrt.getwch()
                if key in ('q','Q','\x1b','\x03'):raise KeyboardInterrupt('User stop')
                if key==' ':start.set()
            if start.is_set():
                if started_at is None:started_at=time.monotonic()
                if time.monotonic()-started_at>45:raise RuntimeError('Camera sweep process exceeded 45s')
            camera.read_packet()
            time.sleep(.02)
        report.update(json.loads((folder/'worker_result.json').read_bytes()))
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc) or 'Interrupted')
    finally:
        if camera is not None:
            try:camera.release()
            except Exception as exc:report.update(status='stopped',camera_cleanup_error=str(exc))
        # Worker may have finished saving after cancellation, during cleanup.
        if (folder/'worker_result.json').is_file():
            saved=json.loads((folder/'worker_result.json').read_bytes())
            if report['status']=='stopped':report['worker_partial_result']=saved
            else:report.update(saved)
        if 'restoration' not in report:
            report['restoration']=report.get('worker_partial_result',{}).get('restoration',
                dict(restored_original_readbacks=False,reason='No completed worker restoration report'))
        payload=json.dumps(report,ensure_ascii=False,indent=2)
        (folder/'result.json').write_text(payload,encoding='utf-8')
        (ROOT/'leap_camera_exposure_sweep_result.json').write_text(payload,encoding='utf-8')
        print('Saved:',folder,flush=True)
        print('No robot movement or motor commands. Camera readbacks/restoration are recorded.',flush=True)
        if report.get('status')=='camera_exposure_diagnosis_complete':
            print('Capture complete. Do not repeat; send just "done" for saved-file review.',flush=True)
            print('Readable palm+cube pair found:',report.get('usable_pair_observed'),flush=True)
        else:print('STOP:',report.get('error','See saved diagnostic result'),flush=True)
    return 0 if report['status']=='camera_exposure_diagnosis_complete' else 1


if __name__=='__main__':raise SystemExit(main())
