import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import cv2 as cv
import numpy as np
from policy_vision_evidence import Evidence


class Tests(unittest.TestCase):
    def test_raw_pixels_corners_and_worst_error_survive_overlay_and_later_save(self):
        evidence=Evidence()
        raw=np.arange(36,dtype=np.uint8).reshape(3,4,3)
        detector=SimpleNamespace(last_ids=np.array([[1]]),last_corners=[np.arange(8).reshape(1,4,2)])
        row=dict(reason='reprojection_over_3px_or_invalid',reprojection_px=3.1)
        with patch.object(Path,'write_bytes',side_effect=AssertionError('No disk during motion')):
            evidence.remember(raw,{},row,detector)
            evidence.remember(raw,{},dict(row,reprojection_px=4.2),detector)
            evidence.remember(raw,{},dict(row,reprojection_px=3.2),detector)
        raw.fill(255); detector.last_corners[0].fill(99)
        camera=SimpleNamespace(K=np.eye(3),dist=np.zeros(5))
        with tempfile.TemporaryDirectory() as folder:
            report={}; evidence.save(folder,cv,camera,report)
            decoded=cv.imdecode(np.frombuffer(Path(report['policy_evidence_images'][0]).read_bytes(),dtype=np.uint8),cv.IMREAD_COLOR)
            np.testing.assert_array_equal(decoded,np.arange(36,dtype=np.uint8).reshape(3,4,3))
            data=json.loads(next(Path(folder).glob('*.json')).read_text())
            self.assertEqual(data['metrics']['reprojection_px'],4.2)
            self.assertEqual(data['geometry']['markers'][0]['corners_px'][0],[0,1])

    def test_memory_is_bounded_and_encode_failure_is_reported(self):
        evidence=Evidence(); frame=np.zeros((2,2,3),dtype=np.uint8)
        for i in range(100):
            evidence.remember(frame,None,dict(reason=str(i),reprojection_px=None),None)
        self.assertEqual(len(evidence.examples),8)
        with tempfile.TemporaryDirectory() as folder:
            report={}
            evidence.save(folder,SimpleNamespace(imencode=Mock(return_value=(False,None))),None,report)
            self.assertEqual(len(report['policy_evidence_save_errors']),8)
            self.assertEqual(report['policy_evidence_images'],[])

    def test_policy_diagnostics_record_reprojection_and_age_without_changing_gate(self):
        from test_loaded_policy_trial import Tests as Loaded
        report,error,_=Loaded().case(radius=65)
        self.assertIsNone(error)
        for row in report['policy_vision_diagnostics']:
            self.assertIn('frame_age_ms',row)
            self.assertIn('reprojection_px',row)
            if row['reason'] is None:
                self.assertLessEqual(row['reprojection_px'],3)
                self.assertLessEqual(row['frame_age_ms'],200)
