"""Verify stage-end camera evidence is saved without opening hardware."""

import json
import tempfile
import threading
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

from sequence_vision_recorder import SequenceVisionRecorder


class PhaseEndTest(unittest.TestCase):
    def test_finish_saves_last_stage_frame(self):
        with tempfile.TemporaryDirectory() as temp:
            recorder = SequenceVisionRecorder.__new__(SequenceVisionRecorder)
            recorder.root = Path(temp)
            recorder.motor_report = {}
            recorder.report = {'frames': [], 'status': 'recording'}
            recorder.stop_event = threading.Event()
            recorder.thread = None
            recorder.error = None
            recorder.camera = SimpleNamespace(cv=cv2, close=lambda: None)
            recorder.examples = {}
            recorder.phase_last = {
                'stage_1': (12.5, np.zeros((8, 8, 3), np.uint8),
                            {'motor_t_s': 2.1, 'detected_ids': [1, 41]}),
                'preflight': (1.0, np.zeros((8, 8, 3), np.uint8),
                              {'motor_t_s': None, 'detected_ids': [40]}),
            }
            recorder.offset = 100.0
            recorder.placement_frame = None
            recorder.anchor_empty_frame = None
            recorder.anchor_preflight_frame = None
            recorder.anchor_cube_candidate = None
            recorder.recent_frames = deque()
            result = recorder.finish()
            report = json.loads(Path(result['result']).read_text(encoding='utf-8'))
            self.assertEqual(report['phase_end_images']['stage_1']['detected_ids'], [1, 41])
            self.assertEqual(report['phase_end_images']['stage_1']['captured_at_unix_s'], 112.5)
            self.assertTrue((Path(result['folder']) / 'stage_1_end.png').is_file())
            self.assertFalse((Path(result['folder']) / 'preflight_end.png').exists())


if __name__ == '__main__':
    unittest.main()
