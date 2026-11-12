"""Camera recording logic without opening a webcam or a GUI."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
import time
from types import SimpleNamespace
import unittest
import numpy as np
from empty_trial_camera import EmptyTrialCamera
from camera_frame_signal import FrameSignal,CameraImageUnavailable

def fake(folder,recording=True):
    x=EmptyTrialCamera.__new__(EmptyTrialCamera)
    x.stop=threading.Event();x.lock=threading.Lock();x.error=None;x.frames=[];x.latest=None
    x.signal=FrameSignal()
    x.report=dict(recording_requested=recording,phase='stage_1_thumb_prefix',enabled_at=10.)
    x.folder=Path(folder);x.started=time.monotonic();x.title='fake'
    x.cv=SimpleNamespace(IMWRITE_JPEG_QUALITY=1,FONT_HERSHEY_SIMPLEX=0,
        imencode=lambda *a:(True,np.array([1,2,3],dtype=np.uint8)),
        putText=lambda *a:None,imshow=lambda *a:None,waitKey=lambda *a:-1)
    calls=[]
    def packet():
        calls.append(1)
        if len(calls)>1:x.stop.set();return None
        return time.monotonic()-.01,np.arange(27,dtype=np.uint8).reshape(3,3,3)
    x.camera=SimpleNamespace(read_packet=packet,preview_stop_requested=False)
    def check_preview(instruction):
        if x.camera.preview_stop_requested:raise KeyboardInterrupt('Camera user stop')
    x.camera.check_preview=check_preview
    return x

class Tests(unittest.TestCase):
    def test_preview_does_not_fill_disk(self):
        with TemporaryDirectory() as folder:
            x=fake(folder,False);x._record()
            self.assertIsNotNone(x.latest);self.assertEqual(x.frames,[])
            self.assertEqual(list(Path(folder).iterdir()),[])

    def test_record_has_timestamp_and_motion_phase(self):
        with TemporaryDirectory() as folder:
            x=fake(folder);x._record()
            self.assertIsNone(x.error);self.assertEqual(len(x.frames),1)
            self.assertEqual(x.frames[0]['phase'],'stage_1_thumb_prefix')
            self.assertEqual((Path(folder)/x.frames[0]['image']).read_bytes(),bytes([1,2,3]))

    def test_image_failure_and_staleness_propagate(self):
        with TemporaryDirectory() as folder:
            x=fake(folder);x.cv.imencode=lambda *a:(False,None);x._record()
            with self.assertRaisesRegex(RuntimeError,'JPEG'):x.check()
            x=fake(folder);x.latest=(time.monotonic()-3,np.zeros((3,3,3)))
            with self.assertRaisesRegex(RuntimeError,'stale'):x.check()

    def test_preview_user_q_and_start_freshness(self):
        with TemporaryDirectory() as folder:
            x=fake(folder);x.camera.preview_stop_requested=True
            with self.assertRaises(KeyboardInterrupt):x.check()
            x=fake(folder)
            with self.assertRaisesRegex(RuntimeError,'fresh preview'):x.require_fresh()

    def test_finish_does_not_claim_success_for_empty_recording(self):
        for recording in (False,True):
            with TemporaryDirectory() as folder:
                x=fake(folder,recording);x._record()
                x.thread=SimpleNamespace(join=lambda **k:None,is_alive=lambda:False)
                x.camera.release=lambda:None;x.cv.destroyAllWindows=lambda:None
                result=x.finish()
                self.assertEqual(result['status'],'complete' if recording else 'incomplete')
                self.assertEqual(len(json.loads((Path(folder)/'frames.json').read_text())['frames']),int(recording))

    def test_black_preview_cannot_start_or_capture(self):
        with TemporaryDirectory() as folder:
            x=fake(folder,False);stamp=time.monotonic()-.01;frame=np.zeros((24,24,3),dtype=np.uint8)
            x.latest=(stamp,frame);x.signal.observe(stamp,frame)
            with self.assertRaises(CameraImageUnavailable):x.require_fresh()
            self.assertIsNone(x.capture_named('must_not_exist.jpg'))
            self.assertFalse((Path(folder).parent/'must_not_exist.jpg').exists())

    def test_blank_during_recording_stops_after_half_second_and_cannot_complete(self):
        with TemporaryDirectory() as folder:
            x=fake(folder);now=time.monotonic();frame=np.zeros((24,24,3),dtype=np.uint8)
            x.signal.observe(now-.8,frame);x.signal.observe(now-.01,frame);x.latest=(now-.01,frame)
            with self.assertRaisesRegex(CameraImageUnavailable,'0.5s'):x.check()
            self.assertIsNotNone(x.error)

    def test_three_distinct_good_packets_recover_preview(self):
        with TemporaryDirectory() as folder:
            x=fake(folder,False);now=time.monotonic()
            frame=np.arange(27,dtype=np.uint8).reshape(3,3,3)
            for delta in (-.2,-.1,-.01):x.signal.observe(now+delta,frame)
            x.latest=(now-.01,frame);x.require_fresh()

if __name__=='__main__':unittest.main()
