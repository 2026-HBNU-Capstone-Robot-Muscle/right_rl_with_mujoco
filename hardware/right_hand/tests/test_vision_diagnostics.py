"""Camera-only capture and diagnosis tests; no physical devices opened."""
import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import cv2 as cv
import numpy as np
import vision_diagnostics as diag
import leap_vision_check as check


def pose(error=1.,markers=2):
    return dict(n_cube_markers=markers,reproj=error,palm_minus_cube=[0,0,0],cube_quat_world=[1,0,0,0])


class Tests(unittest.TestCase):
    def test_metrics_preserve_gate_and_distinguish_failures(self):
        detection=dict(ids=[40,3,5],min_edge_px_by_id={'5':25.4})
        self.assertIsNone(diag.frame_metrics(pose(3),1,detection)['reason'])
        bad=diag.frame_metrics(pose(3.01),1,detection)
        self.assertEqual(bad['reason'],'reprojection_over_3px_or_invalid')
        self.assertIn('3.01px',diag.display_line(bad))
        self.assertEqual(diag.frame_metrics(pose(markers=1),1,detection)['reason'],'cube_markers_below_2')
        self.assertEqual(diag.frame_metrics(None,1,detection)['reason'],'no_pose')
        self.assertIsNone(diag.frame_metrics(pose(float('nan')),1,detection)['reprojection_px'])

    def test_summary_groups_faces_and_consecutive_validity(self):
        rows=[diag.frame_metrics(pose(e),1,dict(ids=[40,5,3])) for e in [1,2,4,1,2,3]]
        result=diag.summarize(rows)
        self.assertEqual(result['eligible_frames'],5)
        self.assertEqual(result['longest_valid_streak'],3)
        self.assertEqual(result['by_cube_ids']['3,5']['max_reprojection_px'],4)
        self.assertEqual(diag.summarize([])['eligible_percent'],0)

    def test_per_marker_projection_identifies_bad_corners(self):
        points=np.array([[-.01,-.01,0],[.01,-.01,0],[.01,.01,0],[-.01,.01,0]],np.float32)
        board=cv.aruco.Board([points],cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),np.array([3]))
        K=np.array([[600.,0,320],[0,600,240],[0,0,1]])
        dist=np.zeros(5); t=np.array([0.,0.,.3]); R=np.eye(3)
        pixels=cv.projectPoints(points,np.zeros(3),t,K,dist)[0].reshape(1,4,2)
        camera=SimpleNamespace(cv=cv,K=K,dist=dist,board=board,
            detector=SimpleNamespace(last_corners=[pixels+np.array([4.,0.],np.float32)],last_ids=np.array([[3]])))
        result=diag.geometry_snapshot(camera,dict(R_cube=R,t_cube=t))
        self.assertAlmostEqual(result['markers'][0]['mean_reprojection_px'],4,places=4)
        self.assertEqual(result['markers'][0]['id'],3)

    def capture(self,close_fails=False,save_fails=False,targeted=False):
        tick=[0.]
        def sleep(seconds): tick[0]+=.1
        class FakeCamera:
            def __init__(self,report):
                self.report=report; self.pressed=False; self.pressed_target=False; self.closed=False
                self.cv=SimpleNamespace(WINDOW_NORMAL=0,namedWindow=lambda *a:None,
                    resizeWindow=lambda *a:None,imencode=cv.imencode,
                    FONT_HERSHEY_SIMPLEX=0,putText=lambda *a:None)
            def next(self,**kwargs): return tick[0],np.full((8,8,3),17,np.uint8)
            def detect(self,frame):
                ids=[40,1,4] if targeted and tick[0]>=2 else [40,3,5]
                self.report['last_marker_detection']=dict(ids=ids)
                return pose(4.)  # SPACE must record even when the policy gate rejects the pose.
            def show(self,frame,lines): frame[:]=255  # Must not contaminate saved raw pixels.
            def key(self):
                if tick[0]>15: raise RuntimeError('Test recording did not finish')
                if not self.pressed: self.pressed=True; return 32
                if targeted and tick[0]>=2 and not self.pressed_target:
                    self.pressed_target=True; return 32
                return 255
            def close(self):
                self.closed=True
                if close_fails: raise RuntimeError('close failed')
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)/'\ud55c\uae00'; root.mkdir()
            with contextlib.ExitStack() as stack:
                stack.enter_context(patch.object(check,'ROOT',root))
                stack.enter_context(patch.object(check,'Camera',FakeCamera))
                stack.enter_context(patch.object(check.time,'monotonic',lambda:tick[0]))
                stack.enter_context(patch.object(check.time,'sleep',sleep))
                stack.enter_context(patch.object(check,'geometry_snapshot',return_value={'markers':[]}))
                stack.enter_context(patch('dynamixel_sdk.PortHandler',side_effect=AssertionError('No motor port allowed')))
                if save_fails: stack.enter_context(patch.object(check,'save_image',side_effect=OSError('disk error')))
                stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
                rc=check.main((1,4) if targeted else None)
            result=json.loads((root/'leap_vision_check_result.json').read_text())
            self.assertFalse(result['motor_port_opened'])
            self.assertEqual(result['policy_actions_sent'],0)
            self.assertGreater(result['summary']['frames'],90)
            self.assertEqual(result['summary']['eligible_frames'],0)
            self.assertGreaterEqual(tick[0],10)
            if not save_fails:
                raw=cv.imdecode(np.frombuffer(Path(result['saved_images'][0]).read_bytes(),np.uint8),cv.IMREAD_COLOR)
                self.assertTrue(np.all(raw==17))
            return rc,result

    def test_invalid_pose_can_be_recorded_without_motors(self):
        rc,result=self.capture()
        self.assertEqual(rc,0)
        self.assertEqual(result['status'],'complete')

    def test_close_failure_preserves_evidence(self):
        rc,result=self.capture(close_fails=True)
        self.assertEqual(rc,1)
        self.assertEqual(result['close_error'],'close failed')
        self.assertEqual(len(result['saved_images']),1)

    def test_save_failure_is_reported(self):
        rc,result=self.capture(save_fails=True)
        self.assertEqual(rc,1)
        self.assertEqual(result['save_errors'],['disk error'])

    def test_requested_faces_gate_start_but_not_pose_quality(self):
        rc,result=self.capture(targeted=True)
        self.assertEqual(rc,0)
        self.assertEqual(result['wrong_target_space_count'],1)
        self.assertEqual(result['required_cube_ids'],[1,4])
        self.assertEqual(result['target_present_frames'],len(result['frames']))
        self.assertTrue(all(set(r['detected_ids'])=={40,1,4} for r in result['frames']))
        self.assertEqual(result['summary']['eligible_frames'],0)  # Bad reprojection is evidence to capture.

    def test_target_requires_hand_marker_and_both_requested_faces(self):
        for ids in ([40,0,2],[1,4],[40,1]):
            self.assertFalse(check.target_status(dict(detected_ids=ids),(1,4))[0])
        self.assertTrue(check.target_status(dict(detected_ids=[40,1,4]),(1,4))[0])

    def test_missing_hand_guidance_does_not_ask_for_cube_rotation(self):
        for ids in ([1,4],[1],[]):
            found,message=check.target_status(dict(detected_ids=ids),(1,4))
            self.assertFalse(found)
            self.assertIn('SHOW HAND ID40',message)
            self.assertNotIn('ROTATE',message)
        found,message=check.target_status(dict(detected_ids=[40,1]),(1,4))
        self.assertFalse(found)
        self.assertIn('ROTATE WHOLE CUBE: missing IDs 4',message)


if __name__=='__main__': unittest.main()
