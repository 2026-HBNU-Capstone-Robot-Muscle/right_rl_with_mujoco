"""GUI stalls/Q/raw-frame invariants with fake worker processes only."""
import contextlib
import io
import os
import queue
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from camera_stream import CameraStream,render_preview
from empty_trial_camera import EmptyTrialCamera
from camera_frame_signal import FrameSignal


def stalled_gui_worker(send,stop,preview):
    # Simulates a native GUI drag call longer than the 500ms motor watchdog.
    # No GUI, webcam, serial port or motor module is opened in this worker.
    send.send(('ready',dict(test_worker_pid=os.getpid(),real_devices_opened=False)))
    send.send(('frame',(time.monotonic(),np.arange(27,dtype=np.uint8).reshape(3,3,3))))
    time.sleep(.85)
    while not stop.wait(.01):
        send.send(('frame',(time.monotonic(),np.arange(27,dtype=np.uint8).reshape(3,3,3))))
    send.close()


def user_q_worker(send,stop,preview):
    send.send(('ready',dict(real_devices_opened=False)))
    send.send(('preview_stop','Q'))
    stop.wait();send.close()


def never_returns_worker(send,stop,preview):
    send.send(('ready',dict(real_devices_opened=False)))
    time.sleep(60)


def make_check_only_camera(stream):
    x=EmptyTrialCamera.__new__(EmptyTrialCamera)
    x.camera=stream;x.error=None;x.lock=threading.Lock();x.signal=FrameSignal()
    stamp=time.monotonic();frame=np.arange(27,dtype=np.uint8).reshape(3,3,3)
    for delta in (-.04,-.03,-.02):x.signal.observe(stamp+delta,frame)
    x.latest=(stamp,frame);x.started=stamp;x.report=dict(recording_requested=True,phase='hold')
    def forbidden(*args,**kwargs):raise AssertionError('GUI called on the control thread')
    x.cv=SimpleNamespace(imshow=forbidden,waitKey=forbidden,putText=forbidden,destroyAllWindows=forbidden)
    return x


class Tests(unittest.TestCase):
    def stream(self,worker):
        with contextlib.redirect_stdout(io.StringIO()):
            return CameraStream({},worker=worker,preview_title='FAKE ONLY',timeout=8.)

    def test_gui_stall_does_not_block_health_poll_or_trip_500ms_watchdog(self):
        stream=self.stream(stalled_gui_worker)
        try:
            x=make_check_only_camera(stream)
            began=time.monotonic();last_health=began;gaps=[];costs=[]
            while time.monotonic()-began<1.:
                before=time.monotonic();x.check();now=time.monotonic()
                costs.append(now-before);gaps.append(now-last_health);last_health=now
                self.assertLess(gaps[-1],.5,'500ms emulated motor watchdog expired')
                time.sleep(.01)
            self.assertGreater(len(costs),40)
            self.assertLess(max(costs),.2,'camera check used the entire fresh-read deadline')
            self.assertNotEqual(stream.meta['test_worker_pid'],os.getpid())
            self.assertFalse(stream.meta['real_devices_opened'])
        finally:stream.release()

    def test_full_instruction_queue_never_blocks_control(self):
        stream=self.stream(stalled_gui_worker)
        try:
            began=time.monotonic()
            for i in range(1000):stream.check_preview('instruction '+str(i))
            self.assertLess(time.monotonic()-began,.2)
        finally:stream.release()

    def test_camera_window_q_reaches_parent_without_gui_polling(self):
        stream=self.stream(user_q_worker)
        try:
            deadline=time.monotonic()+2
            while not stream.preview_stop_requested and time.monotonic()<deadline:time.sleep(.005)
            x=make_check_only_camera(stream)
            with self.assertRaisesRegex(KeyboardInterrupt,'Camera user stop'):x.check()
        finally:stream.release()

    def test_unresponsive_gui_process_has_bounded_cleanup(self):
        stream=self.stream(never_returns_worker)
        began=time.monotonic();stream.release()
        self.assertFalse(stream.process.is_alive())
        self.assertTrue(stream.report['camera_worker_stopped'])
        self.assertLess(time.monotonic()-began,2.5)

    def test_gui_annotations_never_modify_recorded_raw_frame(self):
        frame=np.zeros((12,12,3),np.uint8);messages=queue.Queue();messages.put('latest instruction')
        observed=[]
        def annotate(display,*args):display[:]=255
        cv=SimpleNamespace(FONT_HERSHEY_SIMPLEX=0,WND_PROP_VISIBLE=0,
            putText=annotate,imshow=lambda title,image:observed.append(image.copy()),
            waitKey=lambda ms:ord('q'),getWindowProperty=lambda *args:1)
        text,stop=render_preview(cv,frame,dict(updates=messages,title='FAKE'),'old')
        self.assertEqual(text,'latest instruction');self.assertTrue(stop)
        self.assertTrue(np.all(frame==0));self.assertTrue(np.all(observed[0]==255))

    def test_closed_window_requests_parent_stop(self):
        cv=SimpleNamespace(FONT_HERSHEY_SIMPLEX=0,WND_PROP_VISIBLE=0,
            putText=lambda *args:None,imshow=lambda *args:None,waitKey=lambda ms:-1,
            getWindowProperty=lambda *args:-1)
        _,stop=render_preview(cv,np.zeros((4,4,3),np.uint8),dict(updates=queue.Queue(),title='FAKE'),'hold')
        self.assertTrue(stop)


if __name__=='__main__':unittest.main()
