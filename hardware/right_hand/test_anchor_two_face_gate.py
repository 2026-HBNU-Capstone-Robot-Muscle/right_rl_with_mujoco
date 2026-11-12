"""Motor-free checks for the two-face capture gate."""

import time
import threading
import unittest

import numpy as np

from sequence_vision_recorder import SequenceVisionRecorder


class AnchorTwoFaceGateTest(unittest.TestCase):
    def recorder(self, minimum):
        rec = SequenceVisionRecorder.__new__(SequenceVisionRecorder)
        rec.frame_lock = threading.Lock()
        rec.anchor_min_cube_markers = minimum
        rec.anchor_empty_frame = np.zeros((2, 2, 3), dtype=np.uint8)
        rec.anchor_cube_candidate = None
        return rec

    def test_two_face_accepts_fresh_pair(self):
        rec = self.recorder(2)
        rec.anchor_cube_candidate = (time.monotonic(), np.zeros((2, 2, 3), dtype=np.uint8),
                                     {'detected_ids': [1, 3], 'anchor_detected_ids': [41]})
        row = rec.mark_placement(require_anchor_cube=True)
        self.assertEqual(row['detected_ids'], [1, 3])
        self.assertFalse(row['capture_stale'])

    def test_two_face_rejects_stale_pair(self):
        rec = self.recorder(2)
        rec.anchor_cube_candidate = (time.monotonic() - 1, np.zeros((2, 2, 3), dtype=np.uint8),
                                     {'detected_ids': [1, 3], 'anchor_detected_ids': [41]})
        with self.assertRaisesRegex(RuntimeError, 'Fresh ID41'):
            rec.mark_placement(require_anchor_cube=True)

    def test_two_face_rejects_absent_pair(self):
        rec = self.recorder(2)
        with self.assertRaisesRegex(RuntimeError, 'No ID41'):
            rec.mark_placement(require_anchor_cube=True)


if __name__ == '__main__':
    unittest.main()
