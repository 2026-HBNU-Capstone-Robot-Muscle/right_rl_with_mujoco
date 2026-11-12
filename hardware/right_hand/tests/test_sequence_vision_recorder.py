"""Camera logging stays independent of the motor sequence."""
import json
import tempfile
import time
import unittest
from pathlib import Path

import cv2
import numpy as np

from sequence_vision_recorder import SequenceVisionRecorder
from sequence_flip_checked import SupportPhotoPreview


class FakeCv:
    WINDOW_NORMAL = 0

    def __init__(self):
        self.opened = False
        self.closed = False

    def namedWindow(self, title, flag):
        self.opened = True

    def resizeWindow(self, title, width, height):
        pass

    def destroyWindow(self, title):
        self.closed = True

    def imencode(self, extension, frame):
        return cv2.imencode(extension, frame)


class FakeCamera:
    def __init__(self, report):
        self.report = report
        self.cv = FakeCv()
        self.sent = 0
        self.closed = False

    def next(self, *, preview_only=False):
        if self.sent >= 2:
            return None
        self.sent += 1
        return time.monotonic(), np.zeros((32, 32, 3), dtype=np.uint8)

    def detect(self, frame):
        self.report['last_marker_detection'] = {'ids': [40, 1]}
        return {'n_cube_markers': 1, 'reproj': .4}

    def show(self, frame, lines):
        self.preview_lines = lines

    def key(self):
        return 32

    def close(self):
        self.closed = True


class Tests(unittest.TestCase):
    def test_support_capture_first_space_waits_until_cube_leaves_camera(self):
        class PreviewCamera(FakeCamera):
            def __init__(self, report):
                super().__init__(report)
                self.preview_count = 0
                self.first_space_accepted_after = None

            def next(self, *, preview_only=False):
                if preview_only and not self.cv.closed:
                    self.preview_count += 1
                    if self.preview_count > 14:
                        raise AssertionError('Preview did not accept SPACE after 10 cube-free frames')
                    return time.monotonic(), np.zeros((32, 32, 3), dtype=np.uint8)
                if self.sent:
                    return None
                self.sent = 1
                return time.monotonic(), np.zeros((32, 32, 3), dtype=np.uint8)

            def detect(self, frame):
                # Four SPACE presses occur with a cube in view, then ten without it.
                self.report['last_marker_detection'] = {
                    'ids': [40, 1] if self.preview_count <= 4 else [40]
                }
                return {'n_cube_markers': int(self.preview_count <= 4), 'reproj': .4}

            def show(self, frame, lines):
                self.preview_lines = lines

        with tempfile.TemporaryDirectory() as temp:
            cameras = []
            def camera_factory(report):
                cameras.append(PreviewCamera(report))
                return cameras[-1]
            recorder = SequenceVisionRecorder(temp, {'phase': 'preflight'},
                                              camera_factory=camera_factory,
                                              support_photo_mode=True)
            self.assertEqual(cameras[0].preview_count, 14)
            self.assertIn('10/10', cameras[0].preview_lines[-2])
            recorder.finish()

    def test_support_preview_shows_fresh_frame_and_accepts_camera_space(self):
        class PreviewCv:
            WINDOW_NORMAL = 0
            FONT_HERSHEY_SIMPLEX = 0

            def __init__(self):
                self.shown = False
                self.closed = False

            def namedWindow(self, *args): pass
            def resizeWindow(self, *args): pass
            def line(self, *args): pass
            def putText(self, *args): pass
            def imshow(self, *args): self.shown = True
            def waitKey(self, *args): return 32
            def destroyWindow(self, *args): self.closed = True

        class PreviewRecorder:
            def __init__(self):
                import threading
                self.frame_lock = threading.Lock()
                self.camera = type('Camera', (), {'cv': PreviewCv()})()
                self.last_analyzed = (time.monotonic(), np.zeros((720, 1280, 3), np.uint8),
                                      {'detected_ids': [], 'reason': 'no_pose'})

        recorder = PreviewRecorder()
        preview = SupportPhotoPreview(recorder)
        self.assertTrue(preview.show())
        self.assertTrue(recorder.camera.cv.shown)
        preview.close()
        self.assertTrue(recorder.camera.cv.closed)

    def test_phase_and_motor_clock_are_saved_without_a_motor_bus(self):
        with tempfile.TemporaryDirectory() as temp:
            motor = {'phase': 'stage_2', 'enabled_at': time.monotonic()-1}
            cameras = []
            def camera_factory(report):
                cameras.append(FakeCamera(report))
                return cameras[-1]
            recorder = SequenceVisionRecorder(temp, motor, camera_factory=camera_factory)
            deadline = time.monotonic()+1
            while not recorder.report['frames'] and time.monotonic()<deadline:
                time.sleep(.005)
            recorder.mark_placement()
            motor['status'] = 'stopped'
            motor['samples'] = [{'t': time.monotonic()-motor['enabled_at']}]
            result = recorder.finish()
            self.assertEqual(result['status'], 'complete')
            self.assertTrue(cameras[0].closed)
            self.assertTrue(cameras[0].cv.opened)
            self.assertTrue(cameras[0].cv.closed)
            rows = json.loads(Path(result['result']).read_text(encoding='utf-8'))['frames']
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]['phase'], 'stage_2')
            self.assertEqual(rows[0]['reason'], 'cube_markers_below_2')
            self.assertEqual(rows[0]['detected_ids'], [40, 1])
            self.assertGreater(rows[0]['motor_t_s'], 0)
            self.assertEqual(result['by_phase']['stage_2']['eligible_frames'], 0)
            self.assertTrue((Path(result['folder'])/'stage_2_not_ready.png').is_file())
            self.assertTrue((Path(result['folder'])/'placement_at_space.png').is_file())
            self.assertTrue((Path(result['folder'])/'failure_nearest.png').is_file())

    def test_support_photo_accepts_fresh_frame_without_markers_but_rejects_stale(self):
        with tempfile.TemporaryDirectory() as temp:
            motor = {'phase': 'place_cube'}
            recorder = SequenceVisionRecorder(temp, motor,
                                              camera_factory=lambda report: FakeCamera(report))
            deadline = time.monotonic()+1
            while not recorder.report['frames'] and time.monotonic()<deadline:
                time.sleep(.005)
            with recorder.frame_lock:
                stamp, frame, row = recorder.last_analyzed
                row = dict(row, detected_ids=[], reason='no_pose')
                recorder.last_analyzed = (stamp, frame, row)
            self.assertEqual(recorder.mark_placement(require_support_photo=True)['detected_ids'], [])
            with recorder.frame_lock:
                recorder.last_analyzed = (stamp-2, frame, row)
            with self.assertRaisesRegex(RuntimeError, 'Fresh camera frame'):
                recorder.mark_placement(require_support_photo=True)
            recorder.finish()


if __name__ == '__main__':
    unittest.main()
