import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from leap_sac_start_capture import remember_final_empty


class CaptureVisibilityTests(unittest.TestCase):
    def recorder(self, ids, stamp=10.):
        return SimpleNamespace(frame_lock=threading.Lock(), examples={},
            last_analyzed=(stamp,np.zeros((4,4,3),dtype=np.uint8),{'anchor_detected_ids':ids}),
            anchor_empty_frame='original paired marker frame')

    def test_id40_occlusion_does_not_block_diagnostic_photo(self):
        recorder=self.recorder([41]);report={}
        with patch('leap_sac_start_capture.time.monotonic',return_value=10.1):
            self.assertTrue(remember_final_empty(recorder,report,9.9))
        self.assertEqual(recorder.anchor_empty_frame,'original paired marker frame')
        self.assertFalse(report['registration_verified'])
        self.assertFalse(report['final_empty_reference']['hand_marker_visible'])
        self.assertTrue(report['final_empty_reference']['anchor_visible'])
        self.assertIn(('revised_start_final_empty','diagnostic'),recorder.examples)

    def test_no_markers_is_qualitative_photo_only(self):
        recorder=self.recorder([]);report={}
        with patch('leap_sac_start_capture.time.monotonic',return_value=10.1):
            self.assertTrue(remember_final_empty(recorder,report,9.9))
        self.assertFalse(report['final_empty_reference']['anchor_visible'])
        self.assertFalse(report['registration_verified'])

    def test_old_or_stale_image_cannot_be_used_as_final_pose(self):
        for began,now in [(10.2,10.3),(9.9,11.)]:
            recorder=self.recorder([40,41]);report={}
            with patch('leap_sac_start_capture.time.monotonic',return_value=now):
                self.assertFalse(remember_final_empty(recorder,report,began))
            self.assertEqual(recorder.examples,{})


if __name__=='__main__':unittest.main()
