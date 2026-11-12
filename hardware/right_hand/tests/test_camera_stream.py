"""Spawn real camera-free workers to verify deadlines, timestamps and cleanup."""
import time
import unittest
from unittest.mock import patch
import camera_stream as stream
import camera_720_check as hd


def hanging_worker(send, stop):
    time.sleep(120)


def ready_then_hanging_worker(send, stop):
    send.send(('ready', {'device_name': 'fake'}))
    send.send(('frame', (123., 'fake-frame')))
    time.sleep(120)


def error_then_hanging_worker(send, stop):
    send.send(('error', 'fake camera failure'))
    time.sleep(120)


def exited_worker(send, stop):
    send.close()


class Capture:
    def __init__(self, codec):
        self.codec, self.released = codec, False
    def isOpened(self): return True
    def set(self, *args): return True
    def read(self):
        from types import SimpleNamespace
        return True, SimpleNamespace(shape=(720,1280,3))
    def get(self, prop):
        return sum(ord(c) << (8*i) for i,c in enumerate(self.codec)) if prop == 3 else 30
    def release(self): self.released = True


class CV:
    CAP_DSHOW, CAP_MSMF = 1, 2
    CAP_PROP_FOURCC, CAP_PROP_FRAME_WIDTH, CAP_PROP_FRAME_HEIGHT, CAP_PROP_FPS = 3, 4, 5, 6
    def __init__(self, codec): self.cap, self.calls = Capture(codec), []
    def VideoCapture(self, index, backend):
        self.calls.append((index, backend))
        return self.cap
    def VideoWriter_fourcc(self, *chars):
        return sum(ord(c) << (8*i) for i,c in enumerate(chars))


class CameraStreamTests(unittest.TestCase):
    def test_transient_read_failure_recovers_without_stale_frame(self):
        from unittest.mock import Mock
        from threading import Event
        from types import SimpleNamespace
        frame=SimpleNamespace(shape=(720,1280,3))
        cap=Mock(); cap.read.side_effect=[(False,None),(False,None),(True,frame)]
        stamp,result=stream.read_frame(cap,Event())
        self.assertIs(result,frame)
        self.assertLessEqual(stamp,time.monotonic())
        self.assertEqual(cap.read.call_count,3)

    def test_persistent_read_failure_and_stop_remain_bounded(self):
        from unittest.mock import Mock
        from threading import Event
        stop=Event(); cap=Mock(); cap.read.return_value=(False,None)
        with self.assertRaisesRegex(RuntimeError,'3 consecutive'): stream.read_frame(cap,stop)
        self.assertEqual(cap.read.call_count,3)
        stop.set()
        self.assertIsNone(stream.read_frame(cap,stop))
        self.assertEqual(cap.read.call_count,3)

    def test_name_follows_reordered_indices(self):
        self.assertEqual(stream.select_camera(['Streaming Webcam', 'Laptop']), 0)
        self.assertEqual(stream.select_camera(['Laptop', 'OBS', 'Streaming Webcam']), 2)

    def test_missing_or_duplicate_name_never_selects_another_device(self):
        for names in ([], ['Laptop', 'OBS'], ['Streaming Webcam']*2):
            with self.assertRaisesRegex(RuntimeError, 'No camera selected'):
                stream.select_camera(names)

    def test_strict_dshow_rejects_yuy2_without_msmf(self):
        cv = CV('YUY2')
        with self.assertRaisesRegex(RuntimeError, 'YUY2 fallback rejected'):
            hd.open_hd(cv, 2, require_mjpg=True, backends=((cv.CAP_DSHOW,'DSHOW'),))
        self.assertEqual(cv.calls, [(2, cv.CAP_DSHOW)])
        self.assertTrue(cv.cap.released)

    def test_strict_dshow_accepts_mjpg(self):
        cv = CV('MJPG')
        cap, meta, attempts = hd.open_hd(cv, 2, require_mjpg=True, backends=((cv.CAP_DSHOW,'DSHOW'),))
        self.assertEqual(meta['actual_format'], 'MJPG')
        self.assertEqual(cv.calls, [(2, cv.CAP_DSHOW)])
        self.assertFalse(cap.released)
        cap.release()

    def test_blocked_open_is_terminated_before_return(self):
        report = {}
        began = time.monotonic()
        with self.assertRaisesRegex(RuntimeError, 'startup exceeded'):
            stream.CameraStream(report, timeout=.2, worker=hanging_worker)
        self.assertLess(time.monotonic()-began, 3)
        self.assertTrue(report['camera_worker_stopped'])

    def test_error_or_worker_exit_is_reported_and_closed(self):
        for worker in (error_then_hanging_worker, exited_worker):
            report = {}
            with self.assertRaises(RuntimeError):
                stream.CameraStream(report, timeout=5, worker=worker)
            self.assertTrue(report['camera_worker_stopped'])

    def test_frames_keep_original_time_no_reuse_and_blocked_read_closes(self):
        report = {}
        capture = stream.CameraStream(report, timeout=5, worker=ready_then_hanging_worker)
        try:
            deadline = time.monotonic()+2
            packet = None
            while packet is None and time.monotonic() < deadline:
                packet = capture.read_packet()
                time.sleep(.01)
            self.assertEqual(packet, (123., 'fake-frame'))
            self.assertIsNone(capture.read_packet())
        finally:
            began = time.monotonic()
            capture.release()
        self.assertLess(time.monotonic()-began, 3)
        self.assertTrue(report['camera_worker_stopped'])

    def test_camera_startup_failure_does_not_open_motor_port(self):
        import held_policy_shadow as held
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as folder:
            with patch.object(held, 'Camera', side_effect=RuntimeError('camera failed')):
                with patch.object(held.motion, 'Bus') as bus:
                    with patch.object(held.shadow, 'ROOT', Path(folder)):
                        self.assertEqual(held.main(), 1)
                    bus.assert_not_called()


if __name__ == '__main__': unittest.main()
