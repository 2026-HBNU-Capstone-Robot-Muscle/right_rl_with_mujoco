"""Observation-only grasp uses frames without exposure driver GET/SET calls."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from empty_trial_camera import EmptyTrialCamera
from leap_book_cube_grasp_trial import Camera
from marker_trial_camera import MarkerTrialCamera


class Tests(unittest.TestCase):
    def test_book_camera_selects_standard_frame_worker_without_property_session(self):
        report={};captured={}
        def init(self,folder,r,*,worker=None):
            captured['worker']=worker;self.report=r
        with tempfile.TemporaryDirectory() as tmp,patch.object(EmptyTrialCamera,'__init__',init),patch('marker_trial_camera.make_empty_detector',return_value=object()):
            Camera(Path(tmp)/'camera',report)
        self.assertIsNone(captured['worker'])
        self.assertIsNone(report['camera_requested_exposure'])
        self.assertEqual(report['camera_exposure_control'],'unchanged')
        self.assertFalse(report['marker_visibility_is_motion_gate'])
        self.assertFalse(MarkerTrialCamera.PRESERVE_EXPOSURE)

    def test_finish_does_not_use_stale_exposure_restoration_file_or_claim_verified_restoration(self):
        camera=Camera.__new__(Camera);camera.report={}
        camera.restoration_path=Path('file_that_must_not_be_read.json')
        with patch.object(EmptyTrialCamera,'finish',return_value={'status':'complete'}),patch.object(Path,'is_file',side_effect=AssertionError('Exposure file consulted')):
            result=camera.finish()
        self.assertEqual(result['status'],'complete')
        self.assertFalse(result['exposure_settings_modified'])
        self.assertFalse(result['exposure_restoration_required'])
        self.assertIsNone(result['exposure_restoration_verified'])


if __name__=='__main__':unittest.main()
