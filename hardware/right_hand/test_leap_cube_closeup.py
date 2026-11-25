import unittest
import numpy as np
from leap_cube_closeup import quality

class CloseupQualityTest(unittest.TestCase):
    def check(self,points,ident=5):return quality([np.array(points,np.float32)[None]],np.array([[ident]]),(720,1280,3))
    def test_centered_large_marker(self):
        self.assertTrue(self.check([[590,310],[690,310],[690,410],[590,410]])['ready'])
    def test_small_or_clipped_or_wrong_marker_rejected(self):
        self.assertFalse(self.check([[620,340],[660,340],[660,380],[620,380]])['ready'])
        self.assertFalse(self.check([[10,310],[110,310],[110,410],[10,410]])['ready'])
        self.assertFalse(self.check([[590,310],[690,310],[690,410],[590,410]],41)['ready'])
    def test_missing_duplicate_or_nonfinite_rejected(self):
        self.assertFalse(quality([],None,(720,1280,3))['ready'])
        self.assertFalse(quality([],np.array([[5],[5]]),(720,1280,3))['ready'])
        self.assertFalse(self.check([[np.nan,310],[690,310],[690,410],[590,410]])['ready'])

if __name__=='__main__':unittest.main()
