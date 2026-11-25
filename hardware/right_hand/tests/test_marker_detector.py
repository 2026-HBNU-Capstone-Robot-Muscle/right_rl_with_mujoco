import unittest
from unittest.mock import Mock
import cv2 as cv
import numpy as np
from marker_detector import MarkerDetector


class Tests(unittest.TestCase):
    def setUp(self):
        self.dictionary=cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        self.detector=MarkerDetector(self.dictionary,range(6))

    def test_real_decoder_finds_small_markers_and_drops_unrelated_ids(self):
        gray=np.full((240,480),235,dtype=np.uint8)
        for ident,x in ((0,30),(1,130),(40,230),(10,330)):
            gray[80:116,x:x+36]=cv.aruco.generateImageMarker(self.dictionary,ident,36)
        _,ids,_=self.detector.detectMarkers(gray)
        self.assertEqual(set(ids.ravel()),{0,1,40})
        self.assertGreater(self.detector.stats['min_edge_px_by_id']['0'],30)
        _,ids,_=self.detector.detectMarkers(np.full_like(gray,255))
        self.assertIsNone(ids)  # Never reuse markers from the preceding frame.

    def test_contrast_pass_uses_only_decoded_ids_from_one_pass(self):
        corner=np.array([[[10.,10.],[40.,10.],[40.,40.],[10.,40.]]],np.float32)
        self.detector.detector=Mock()
        self.detector.detector.detectMarkers.side_effect=[
            ([corner]*2,np.array([[0],[40]]),[]),
            ([corner]*3,np.array([[0],[1],[40]]),[])]
        _,ids,_=self.detector.detectMarkers(np.full((60,60),120,dtype=np.uint8))
        self.assertEqual(set(ids.ravel()),{0,1,40})
        self.assertEqual(self.detector.stats['method'],'contrast')

    def test_duplicate_ids_cannot_count_as_two_markers(self):
        corners=[np.zeros((1,4,2),np.float32)]*3
        result=self.detector.filtered((corners,np.array([[1],[1],[40]]),[]))
        self.assertEqual(result[1].ravel().tolist(),[40])
        self.assertFalse(self.detector.score(result)[0])

    def test_no_contrast_pass_when_original_frame_is_complete(self):
        corner=np.zeros((1,4,2),np.float32)
        self.detector.detector=Mock()
        self.detector.dense_detector=Mock()
        self.detector.strong_detector=Mock()
        self.detector.raw_dense_detector=Mock()
        self.detector.detector.detectMarkers.return_value=([corner]*3,np.array([[0],[1],[40]]),[])
        self.detector.detectMarkers(np.full((60,60),120,dtype=np.uint8))
        self.detector.detector.detectMarkers.assert_called_once()
        self.detector.dense_detector.detectMarkers.assert_not_called()
        self.detector.strong_detector.detectMarkers.assert_not_called()
        self.detector.raw_dense_detector.detectMarkers.assert_not_called()

    def test_raw_dense_recovers_before_contrast_without_combining_passes(self):
        corner=np.zeros((1,4,2),np.float32)
        self.detector.detector=Mock()
        self.detector.detector.detectMarkers.return_value=([corner]*2,np.array([[40],[1]]),[])
        self.detector.raw_dense_detector=Mock()
        self.detector.raw_dense_detector.detectMarkers.return_value=([corner]*3,np.array([[40],[4],[1]]),[])
        self.detector.clahe=Mock()
        _,ids,_=self.detector.detectMarkers(np.full((60,60),120,dtype=np.uint8))
        self.assertEqual(set(ids.ravel()),{40,4,1})
        self.assertEqual(self.detector.stats['method'],'raw_dense_threshold')
        self.detector.clahe.apply.assert_not_called()

    def test_saved_sac_missing_id4_is_decoded_from_current_frame(self):
        from pathlib import Path
        from held_policy_shadow import Camera
        from encoder_policy_shadow import pose_rejection
        path=Path(__file__).resolve().parents[1]/'leap_sac_vision_20261005_145409_279464/cube_markers_below_2.png'
        if not path.exists():self.skipTest('Saved SAC evidence unavailable')
        camera=object.__new__(Camera);camera.cv=cv;camera.report={};camera._setup_tracking()
        frame=cv.imdecode(np.frombuffer(path.read_bytes(),dtype=np.uint8),cv.IMREAD_COLOR)
        pose=camera.detect(frame)
        self.assertIsNone(pose_rejection(pose,0))
        self.assertEqual(camera.detector.stats['method'],'raw_dense_threshold')
        self.assertEqual(set(camera.detector.stats['ids']),{40,1,4})

    def test_fourth_pass_uses_unique_decoded_ids_without_weakening_filter(self):
        corner=np.zeros((1,4,2),np.float32)
        incomplete=([corner]*2,np.array([[1],[40]]),[])
        self.detector.detector=Mock()
        self.detector.detector.detectMarkers.return_value=incomplete
        self.detector.dense_detector=Mock()
        self.detector.dense_detector.detectMarkers.return_value=incomplete
        self.detector.strong_detector=Mock()
        self.detector.strong_detector.detectMarkers.return_value=([corner]*3,np.array([[1],[4],[40]]),[])
        _,ids,_=self.detector.detectMarkers(np.full((60,60),120,dtype=np.uint8))
        self.assertEqual(set(ids.ravel()),{1,4,40})
        self.assertEqual(self.detector.stats['method'],'strong_threshold')
        self.detector.strong_detector.detectMarkers.return_value=([corner]*4,np.array([[1],[1],[10],[40]]),[])
        _,ids,_=self.detector.detectMarkers(np.full((60,60),120,dtype=np.uint8))
        self.assertEqual(set(ids.ravel()),{1,40})
        self.assertEqual(self.detector.stats['method'],'contour')

    def test_saved_motion_frame_recovers_without_accepting_false_marker_or_missing_hand(self):
        from pathlib import Path
        from held_policy_shadow import Camera
        from encoder_policy_shadow import pose_rejection
        root=Path(__file__).resolve().parents[1]
        cases=[('leap_loaded_policy_010_vision_20261005_005729_600575/reprojection_over_3px_or_invalid.png',True),
               ('leap_loaded_policy_010_vision_20261005_005729_600575/eligible.png',True),
               ('vision_check_20261004_195458_004025/reprojection_over_3px_or_invalid.png',False),
               ('vision_check_20261005_001844_859269/hand_cube_position_missing.png',False)]
        if not all((root/name).exists() for name,_ in cases):
            self.skipTest('Saved hardware evidence is unavailable')
        camera=object.__new__(Camera); camera.cv=cv; camera.report={}
        camera._setup_tracking()  # Calibration only; no camera stream or motor port.
        for name,eligible in cases:
            with self.subTest(name=name):
                frame=cv.imdecode(np.frombuffer((root/name).read_bytes(),dtype=np.uint8),cv.IMREAD_COLOR)
                pose=camera.detect(frame)
                self.assertEqual(pose_rejection(pose,0) is None,eligible)
                self.assertEqual(camera.detector.stats['corner_refinement'],'contour')

    def test_dense_pass_recovers_current_frame_without_merging_partial_passes(self):
        corner=np.array([[[10.,10.],[40.,10.],[40.,40.],[10.,40.]]],np.float32)
        self.detector.detector=Mock()
        self.detector.dense_detector=Mock()
        self.detector.detector.detectMarkers.side_effect=[
            ([corner]*2,np.array([[0],[40]]),[]),
            ([corner]*2,np.array([[2],[40]]),[])]
        self.detector.dense_detector.detectMarkers.return_value=([corner]*3,np.array([[0],[2],[40]]),[])
        _,ids,_=self.detector.detectMarkers(np.full((60,60),120,dtype=np.uint8))
        self.assertEqual(set(ids.ravel()),{0,2,40})
        self.assertEqual(self.detector.stats['method'],'dense_threshold')
        self.detector.detector.detectMarkers.side_effect=[
            ([corner]*2,np.array([[0],[40]]),[]),
            ([corner]*2,np.array([[2],[40]]),[])]
        self.detector.dense_detector.detectMarkers.return_value=([corner]*2,np.array([[2],[40]]),[])
        _,ids,_=self.detector.detectMarkers(np.full((60,60),120,dtype=np.uint8))
        self.assertEqual(set(ids.ravel()),{0,40})
        self.assertFalse(self.detector.score(([],ids,[]))[0])
