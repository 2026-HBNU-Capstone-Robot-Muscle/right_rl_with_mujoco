"""Raw 10 fps image sequence for short empty-hand trials; no motor interface."""
import json
import threading
import time
from pathlib import Path
from camera_frame_signal import FrameSignal,CameraImageUnavailable

class EmptyTrialCamera:
    def __init__(self,folder,report,*,worker=None):
        import cv2
        from camera_stream import CameraStream
        self.cv=cv2;self.report=report;self.folder=Path(folder);self.folder.mkdir(exist_ok=True)
        self.title='LEAP TRIAL | SPACE in TERMINAL | Q stops'
        options=dict(preview_title=self.title)
        if worker is not None:options['worker']=worker
        self.camera=CameraStream(report,**options)
        self.stop=threading.Event();self.lock=threading.Lock()
        self.latest=None;self.error=None;self.frames=[];self.started=time.monotonic()
        self.signal=FrameSignal()
        self.thread=threading.Thread(target=self._record,daemon=True);self.thread.start()

    def _record(self):
        last_saved=-1.;last_stamp=-1.
        try:
            while not self.stop.is_set():
                packet=self.camera.read_packet()
                if packet is None or packet[0]<=last_stamp:
                    self.stop.wait(.005);continue
                stamp,frame=packet;last_stamp=stamp
                if not 0<=time.monotonic()-stamp<=.3:continue
                with self.lock:
                    quality=self.signal.observe(stamp,frame)
                    self.latest=(stamp,frame.copy())
                if not self.report.get('recording_requested'):continue
                if stamp-last_saved<.1:continue
                ok,jpg=self.cv.imencode('.jpg',frame,[self.cv.IMWRITE_JPEG_QUALITY,90])
                if not ok:raise RuntimeError('Camera JPEG encode failed')
                name=f'frame_{len(self.frames):05d}.jpg'
                (self.folder/name).write_bytes(jpg.tobytes())
                self.frames.append(dict(image=name,camera_read_monotonic=stamp,
                    phase=self.report.get('phase','preview'),enabled_at=self.report.get('enabled_at'),
                    image_signal_usable=quality['usable'],image_signal_reason=quality['reason']))
                last_saved=stamp
        except Exception as exc:self.error=str(exc)

    def check(self):
        if self.error:raise RuntimeError('Camera recording failed: '+self.error)
        with self.lock:
            packet=self.latest
            quality=self.signal.latest
            prolonged=self.signal.prolonged_blank(time.monotonic())
        self.report['camera_image_signal']=quality
        if packet is None:
            if time.monotonic()-self.started>5:raise RuntimeError('No fresh camera frame; no motion')
        else:
            stamp,_=packet
            if time.monotonic()-stamp>2:raise RuntimeError('Camera stale; stop motion')
            if self.report.get('recording_requested') and prolonged:
                self.error='Camera image stayed black/blank for 0.5s; stop motion and check the webcam preview'
                raise CameraImageUnavailable(self.error)
        instruction=self.report.get('camera_instruction','EMPTY HAND')
        if quality is not None and not quality['usable']:
            instruction='CAMERA BLACK/BLANK: check lens and webcam image; SPACE blocked'
        self.camera.check_preview(instruction+' | '+self.report.get('phase','preview'))

    def require_fresh(self):
        self.check()
        with self.lock:packet=self.latest;ready=self.signal.ready()
        if packet is None or time.monotonic()-packet[0]>.3:raise RuntimeError('Wait for a fresh preview; no motor writes')
        if not ready:raise CameraImageUnavailable('Camera image is blank or not stable yet. Show the hand in preview, then press SPACE again. No motor writes.')

    def capture_named(self,name,*,after=0.):
        """Save a fresh raw diagnostic photo, without a marker prerequisite."""
        if Path(name).name!=name:raise ValueError('Snapshot name must be a filename')
        with self.lock:
            packet=None if self.latest is None else (self.latest[0],self.latest[1].copy())
            ready=self.signal.ready()
        if packet is None:return None
        if not ready:return None
        stamp,frame=packet
        if stamp<after or not 0<=time.monotonic()-stamp<=.3:return None
        ok,jpg=self.cv.imencode('.jpg',frame,[self.cv.IMWRITE_JPEG_QUALITY,95])
        if not ok:raise RuntimeError('Placement JPEG encoding failed')
        path=self.folder.parent/name;path.write_bytes(jpg.tobytes())
        return dict(image=name,camera_read_monotonic=stamp,registration_verified=False)

    def finish(self):
        self.stop.set();self.thread.join(timeout=3)
        self.camera.release()
        usable=sum(bool(f.get('image_signal_usable')) for f in self.frames)
        last_usable=self.signal.latest is not None and self.signal.latest['usable']
        status='complete' if not self.error and not self.thread.is_alive() and usable and last_usable else 'incomplete'
        result=dict(status=status,error=self.error,frames=list(self.frames),
            timestamps='Camera driver-read timestamps, not synchronized exposure times.',motor_commands_sent=0,
            image_signal_usable_frames=usable,image_signal_blank_frames=len(self.frames)-usable,
            last_image_signal_usable=bool(last_usable))
        (self.folder/'frames.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        return dict(folder=str(self.folder),status=status,frame_count=len(self.frames),error=self.error,
                    image_signal_usable_frames=usable,image_signal_blank_frames=len(self.frames)-usable,
                    last_image_signal_usable=bool(last_usable))
