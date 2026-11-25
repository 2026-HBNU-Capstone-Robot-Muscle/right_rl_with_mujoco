"""Driver-independent content guard tests, including duplicate packets."""
import unittest
import numpy as np
from camera_frame_signal import FrameSignal,frame_signal


class Tests(unittest.TestCase):
    def test_uniform_black_gray_and_invalid_frames_are_not_scene_signal(self):
        for a in (np.zeros((24,24,3),dtype=np.uint8),np.full((24,24,3),127,dtype=np.uint8),
                  np.zeros((24,24)),np.full((24,24,3),np.nan)):
            with self.subTest(shape=a.shape):self.assertFalse(frame_signal(a)['usable'])

    def test_dim_but_textured_hand_scene_is_not_rejected_for_mean_brightness(self):
        a=np.zeros((24,24,3),dtype=np.uint8);a[::8,::8]=np.arange(9).reshape(3,3,1)*2
        self.assertTrue(frame_signal(a)['usable'])

    def test_repeated_same_packet_cannot_build_ready_streak(self):
        signal=FrameSignal();a=np.arange(27,dtype=np.uint8).reshape(3,3,3)
        for _ in range(5):signal.observe(1.,a)
        self.assertFalse(signal.ready());self.assertEqual(signal.good_streak,1)
        signal.observe(2.,a);signal.observe(3.,a);self.assertTrue(signal.ready())

    def test_single_blank_packet_resets_streak_but_not_immediately_prolonged(self):
        signal=FrameSignal();a=np.arange(27,dtype=np.uint8).reshape(3,3,3)
        for t in (1.,2.,3.):signal.observe(t,a)
        signal.observe(4.,np.zeros_like(a))
        self.assertFalse(signal.ready());self.assertFalse(signal.prolonged_blank(4.49))
        self.assertTrue(signal.prolonged_blank(4.5))
        signal.observe(4.6,a);self.assertFalse(signal.prolonged_blank(4.7))
        self.assertFalse(signal.ready())


if __name__=='__main__':unittest.main()
