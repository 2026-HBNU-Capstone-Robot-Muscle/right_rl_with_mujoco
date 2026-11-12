"""Disposable camera-only worker for explicitly selected measured exposure.

This leaf module does not change the camera used by any motor launcher.
Original property readbacks and restoration are saved before native cleanup.
"""
import json
import math
from pathlib import Path
import time

from camera_exposure_sweep import ExposureSession

EXPOSURE = -7.


def require_exposure(value, expected=EXPOSURE):
    if not math.isfinite(value) or abs(value-expected) > .25:
        raise RuntimeError(f'Camera exposure changed: expected {expected:g}, read {value:g}')


def fixed_exposure_worker(send, stop, preview=None, *, restoration_path,display_mode='readonly',exposure=EXPOSURE,
                          confirm_property_reads=False):
    cap = cv = session = None
    record = dict(status='starting',requested_exposure=exposure,
                  auto_mode_meaning_verified=False,motors_accessed=False,
                  frames_sent=0)
    try:
        if display_mode not in ('readonly','record'):raise ValueError('Unsupported camera display mode')
        if exposure not in (-5.,-6.,-7.):raise ValueError('Unmeasured camera exposure profile')
        import cv2 as cv
        from camera_stream import enumerate_names,select_camera,read_frame,render_preview
        from camera_720_check import open_hd
        from camera_property_confirmation import PropertyReadConfirmation
        from cube_marker_readiness import make_detector,preview_with_markers
        names=enumerate_names();send.send(('devices',names));index=select_camera(names)
        send.send(('selected',dict(camera_index=index,device_name=names[index])))
        cap,meta,attempts=open_hd(cv,index,require_mjpg=True,backends=((cv.CAP_DSHOW,'DSHOW'),))
        session=ExposureSession(cap,cv.CAP_PROP_AUTO_EXPOSURE,cv.CAP_PROP_EXPOSURE)
        record['original_settings']=session.original
        applied=session.apply(exposure);record['applied']=applied
        confirmation=PropertyReadConfirmation(record) if confirm_property_reads else None
        # The setters must succeed. Only an anomalous readback may receive
        # bounded additional READs; the exposure is never set a second time.
        if (confirmation is not None and not applied['supported']
            and applied.get('set_auto_returned') and applied.get('set_exposure_returned')):
            confirmed=confirmation.read(cap,cv.CAP_PROP_EXPOSURE,exposure,stop=stop,
                initial=applied['readback']['exposure'])
            applied['confirmed_exposure_readback']=confirmed;applied['supported']=True
        if not applied['supported']:
            raise RuntimeError(f'Measured {exposure:g} exposure profile is unsupported on this camera; no capture started')
        # Flush six actual driver reads after changing exposure, as in the sweep.
        for _ in range(6):
            if read_frame(cap,stop) is None:raise KeyboardInterrupt('Camera user stop')
        if confirmation is not None:confirmation.read(cap,cv.CAP_PROP_EXPOSURE,exposure,stop=stop)
        else:require_exposure(float(cap.get(cv.CAP_PROP_EXPOSURE)),exposure)
        if preview is not None:
            cv.namedWindow(preview['title'],cv.WINDOW_NORMAL)
            cv.resizeWindow(preview['title'],1000,600)
        # Read-only pose guidance needs marker overlays. A motor trial already
        # checks EMPTY markers in its parent process; decoding a second time
        # here only duplicates CPU work while the native path screen runs.
        detector=make_detector() if preview is not None and display_mode=='readonly' else None
        record['preview_marker_decoding']=detector is not None
        readable_streak=0
        send.send(('ready',dict(meta,camera_index=index,device_name=names[index],
            attempts=[dict(row) for row in attempts],exposure_profile=applied)))
        instruction='READ ONLY: cube/hand stay still; SPACE in terminal; Q exits'
        record['status']='streaming'
        while not stop.is_set():
            packet=read_frame(cap,stop)
            if packet is None:break
            if confirmation is not None:actual=confirmation.read(cap,cv.CAP_PROP_EXPOSURE,exposure,stop=stop)
            else:
                actual=float(cap.get(cv.CAP_PROP_EXPOSURE));require_exposure(actual,exposure)
            record['last_stream_exposure_readback']=actual
            record['frames_sent']+=1
            send.send(('frame',packet))
            if preview is not None:
                display=packet[1]
                if detector is not None:
                    display,readable_streak,quality=preview_with_markers(packet[1],detector,readable_streak,
                        prompt_for_readiness=True)
                    record['last_marker_readiness']=quality
                instruction,requested_stop=render_preview(cv,display,preview,instruction)
                if requested_stop:
                    record['status']='user_stop';send.send(('preview_stop','Camera user stop'));break
        if record['status']=='streaming':record['status']='closed'
    except (Exception,KeyboardInterrupt) as exc:
        record.update(status='stopped',error=str(exc) or 'Interrupted')
        try:send.send(('error',record['error']))
        except (BrokenPipeError,EOFError,OSError):pass
    finally:
        if session is not None:
            try:
                restored=session.restore();record['restoration']=restored
                if (confirm_property_reads and not restored['restored_original_readbacks']
                    and not restored.get('errors') and all(restored.get('set_returns',{}).values())):
                    # A failed first restoration readback is not labelled
                    # restored without fresh confirmation of BOTH properties.
                    from camera_property_confirmation import PropertyReadConfirmation
                    confirm_restore=PropertyReadConfirmation(record)
                    observed={name:confirm_restore.read(cap,prop,session.original[name],label='restore_'+name,
                        initial=restored.get('readback',{}).get(name))
                        for name,prop in (('exposure',session.exposure_property),('auto',session.auto_property))}
                    restored['confirmation_readbacks']=observed
                    restored['restored_original_readbacks']=True
            except Exception as exc:
                record.setdefault('restoration',{}).update(restored_original_readbacks=False,error=str(exc))
        else:
            record['restoration']=dict(restored_original_readbacks=False,
                reason='Camera property session was not established')
        record['finished_monotonic']=time.monotonic()
        try:
            destination=Path(restoration_path);pending=destination.with_suffix('.pending')
            pending.write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
            pending.replace(destination)
        finally:
            # A blocked native release can be terminated by CameraStream. The
            # restoration report has already been written independently of IPC.
            try:
                if cap is not None:cap.release()
            finally:
                if cv is not None and preview is not None:
                    try:cv.destroyAllWindows()
                    except Exception:pass
                send.close()
