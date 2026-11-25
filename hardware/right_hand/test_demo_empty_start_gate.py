"""Saved frames and fake preview only: prevent the observed loaded start."""
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import cv2 as cv
import numpy as np

from camera_frame_signal import FrameSignal,CameraImageUnavailable
from empty_start_marker_check import make_empty_detector,inspect_empty_frame,EmptyStartEvidence
from empty_trial_camera import EmptyTrialCamera
from marker_trial_camera import MarkerTrialCamera,EMPTY_SCREEN_FRAMES

ROOT=Path(__file__).resolve().parent


def fixture_cube():
    path=ROOT/'slow_grasp_demo_20261023_175232_779550/camera/frame_00000.jpg'
    return cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR)


def fake(frame,report=None):
    camera=MarkerTrialCamera.__new__(MarkerTrialCamera)
    camera.report={'phase':'preview'} if report is None else report
    camera.lock=threading.Lock();camera.latest=None;camera.error=None
    camera.signal=FrameSignal();camera.started=time.monotonic()
    camera.camera=SimpleNamespace(check_preview=lambda _:None)
    camera.empty_detector=make_empty_detector();camera.empty_evidence=EmptyStartEvidence()
    camera.cv=SimpleNamespace(imencode=lambda *args:(False,None))
    camera.empty_checked_stamp=None
    camera.empty_streak=0;camera.empty_quality=None
    camera.frame=frame
    return camera


def feed(camera,stamp):
    camera.latest=(stamp,camera.frame)
    camera.signal.observe(stamp,camera.frame)
    camera.check()


class Tests(unittest.TestCase):
    def test_offline_planner_can_import_gate_without_camera_library(self):
        import importlib.util
        spec=importlib.util.spec_from_file_location('empty_gate_without_camera',ROOT/'empty_start_marker_check.py')
        module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,{'cv2':None}):spec.loader.exec_module(module)
        self.assertEqual(module.EmptyStartEvidence().observe(1.,[])['confirmed_cube_ids'],[])

    def test_saved_cube_start_is_blocked_without_needing_palm_marker(self):
        frame=fixture_cube();original=frame.copy();camera=fake(frame)
        now=time.monotonic()
        for delta in (-.2,-.1,0):feed(camera,now+delta)
        with self.assertRaisesRegex(CameraImageUnavailable,'Possible cube marker'):
            camera.require_fresh()
        self.assertIn(5,camera.report['empty_start_marker_screen']['decoded_cube_ids'])
        self.assertEqual(camera.empty_streak,0)
        self.assertFalse(camera.report.get('empty_start_marker_screen_accepted',False))
        np.testing.assert_array_equal(frame,original)

    def test_repeated_packet_cannot_fake_ten_new_frames(self):
        camera=fake(fixture_cube())
        with patch('marker_trial_camera.inspect_empty_frame',return_value=[]):
            stamp=time.monotonic();feed(camera,stamp)
            for _ in range(20):camera.check()
            self.assertEqual(camera.empty_streak,1)
            with self.assertRaises(CameraImageUnavailable):camera.require_fresh()

    def test_removal_recovers_on_fresh_frames_and_missing_palm_does_not_block(self):
        camera=fake(fixture_cube());stamp=time.monotonic()
        feed(camera,stamp-.2)
        with patch('marker_trial_camera.inspect_empty_frame',return_value=[]):
            for i in range(EMPTY_SCREEN_FRAMES):feed(camera,stamp-.1+i*.01)
            camera.require_fresh()
        self.assertTrue(camera.report['empty_start_marker_screen_accepted'])
        self.assertFalse(camera.report['empty_start_marker_screen']['absence_proves_empty_hand'])

    def test_cube_introduced_during_off_screen_or_countdown_stops(self):
        for phase in ('preflight','stable_off_readings','screen_current_pose','countdown'):
            camera=fake(fixture_cube());camera.report['phase']=phase
            with self.subTest(phase=phase),self.assertRaisesRegex(CameraImageUnavailable,'no motion started'):
                stamp=time.monotonic()
                for delta in (-.2,-.1,0):feed(camera,stamp+delta)

    def test_no_decoding_during_configuration_or_motion_or_placement(self):
        for phase in ('configure_off','enable_hold','stage_0_empty_recover','place_cube',
                      'stage_3_loaded_approach','stage_4_fixed_grasp_observation','post_off_reference'):
            camera=fake(fixture_cube());camera.report['phase']=phase
            with self.subTest(phase=phase),patch('marker_trial_camera.inspect_empty_frame',side_effect=AssertionError('ON decode')):
                feed(camera,time.monotonic())
        camera=fake(fixture_cube());camera.report['enabled_at']=1.
        with patch('marker_trial_camera.inspect_empty_frame',side_effect=AssertionError('ON decode')):
            feed(camera,time.monotonic())

    def test_stale_empty_frame_cannot_authorize(self):
        camera=fake(fixture_cube());stamp=time.monotonic()-1
        camera.latest=(stamp,camera.frame);camera.empty_streak=10
        camera.empty_quality=dict(decoded_cube_ids=[],confirmed_cube_ids=[],consecutive_frames_without_decoded_cube=10)
        for delta in (-.2,-.1,0):camera.signal.observe(stamp+delta,camera.frame)
        with self.assertRaisesRegex(RuntimeError,'fresh preview'):camera.require_fresh()

    def test_new_raw_frame_does_not_refresh_an_old_marker_result(self):
        camera=fake(fixture_cube());now=time.monotonic()
        for delta in (-.2,-.1,0):camera.signal.observe(now+delta,camera.frame)
        camera.latest=(now,camera.frame)
        camera.empty_quality=dict(camera_read_monotonic=now-1,decoded_cube_ids=[],
            consecutive_frames_without_decoded_cube=10)
        with patch.object(camera,'check',return_value=None),self.assertRaisesRegex(CameraImageUnavailable,'fresh EMPTY-start'):
            camera.require_fresh()

    def test_first_space_with_saved_cube_never_constructs_motor_bus(self):
        import leap_thumb_prepare_current_empty as main
        from slow_grasp_demo_plan import SlowGraspDemoPlan
        keys=iter((' ','q'))
        def init(obj,folder,report):
            source=fake(fixture_cube(),report);obj.__dict__.update(source.__dict__)
            now=time.monotonic()
            for delta in (-.2,-.1,0):feed(obj,now+delta)
        with tempfile.TemporaryDirectory() as temp,contextlib.redirect_stdout(io.StringIO()), \
             patch.object(main,'ROOT',Path(temp)),patch.object(sys,'argv',['demo','--execute']), \
             patch.dict(sys.modules,{'msvcrt':SimpleNamespace(kbhit=lambda:True,getwch=lambda:next(keys))}), \
             patch.object(MarkerTrialCamera,'__init__',init), \
             patch.object(MarkerTrialCamera,'finish',return_value={'status':'complete'}), \
             patch.object(main,'Bus',side_effect=AssertionError('Motor bus constructed')) as bus:
            self.assertEqual(main.main(SlowGraspDemoPlan),1)
            bus.assert_not_called()
            result=json.loads((Path(temp)/'leap_slow_grasp_demo_result.json').read_bytes())
        self.assertFalse(result['hardware_authorized'])
        self.assertNotIn('enabled_at',result)
        self.assertEqual(result['empty_start_marker_screen']['decoded_cube_ids'],[5])

    def test_all_245_saved_empty_frames_do_not_confirm_a_cube(self):
        folder=ROOT/'slow_grasp_demo_20261023_180831_999579'
        detector=make_empty_detector();evidence=EmptyStartEvidence();candidates=0
        rows=json.loads((folder/'camera/frames.json').read_bytes())['frames']
        self.assertEqual(len(rows),245)
        for row in rows:
            frame=cv.imdecode(np.frombuffer((folder/'camera'/row['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
            found=inspect_empty_frame(frame,detector);candidates+=bool(found)
            result=evidence.observe(row['camera_read_monotonic'],found)
            self.assertEqual(result['confirmed_cube_ids'],[],row['image'])

    def test_saved_real_cube_is_confirmed_across_actual_frame_timestamps(self):
        folder=ROOT/'slow_grasp_demo_20261023_175232_779550'
        detector=make_empty_detector();evidence=EmptyStartEvidence();confirmed=[]
        rows=json.loads((folder/'camera/frames.json').read_bytes())['frames'][:20]
        for row in rows:
            frame=cv.imdecode(np.frombuffer((folder/'camera'/row['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
            result=evidence.observe(row['camera_read_monotonic'],inspect_empty_frame(frame,detector))
            confirmed.extend(result['confirmed_cube_ids'])
        self.assertIn(5,confirmed)

    def test_single_transient_id_cannot_stop_off_screen(self):
        camera=fake(fixture_cube());camera.report['phase']='screen_current_pose'
        stamp=time.monotonic();feed(camera,stamp-.1)
        self.assertEqual(camera.empty_quality['confirmed_cube_ids'],[])
        with patch('marker_trial_camera.inspect_empty_frame',return_value=[]):feed(camera,stamp)
        self.assertEqual(camera.empty_quality['confirmed_cube_ids'],[])
        self.assertEqual(camera.empty_quality['decoded_cube_ids'],[])

    def test_repeated_calls_or_gap_or_changed_location_do_not_confirm(self):
        candidate=dict(id=5,minimum_edge_px=30.,center_px=[100.,100.],corners=[[0,0]]*4)
        evidence=EmptyStartEvidence()
        evidence.observe(1.,[candidate])
        self.assertIsNone(evidence.observe(1.,[candidate]))
        self.assertEqual(evidence.observe(1.1,[candidate])['confirmed_cube_ids'],[])
        self.assertEqual(evidence.observe(1.6,[candidate])['confirmed_cube_ids'],[])
        shifted=dict(candidate,center_px=[300.,100.])
        self.assertEqual(evidence.observe(1.7,[shifted])['confirmed_cube_ids'],[])
        self.assertEqual(evidence.observe(1.8,[shifted])['confirmed_cube_ids'],[])
        self.assertEqual(evidence.observe(1.9,[shifted])['confirmed_cube_ids'],[5])

    def test_confirmed_stop_saves_exact_raw_png_and_marker_geometry(self):
        camera=fake(fixture_cube());camera.report['phase']='screen_current_pose'
        camera.cv=cv;stamp=time.monotonic()
        with tempfile.TemporaryDirectory() as folder:
            camera.folder=Path(folder)/'camera';camera.folder.mkdir()
            feed(camera,stamp-.2);feed(camera,stamp-.1)
            with self.assertRaises(CameraImageUnavailable):feed(camera,stamp)
            saved=cv.imdecode(np.frombuffer((Path(folder)/'empty_start_blocked.png').read_bytes(),np.uint8),cv.IMREAD_COLOR)
            np.testing.assert_array_equal(saved,camera.frame)
            self.assertEqual(camera.report['empty_start_blocked_frame']['camera_read_monotonic'],stamp)
            self.assertTrue(camera.report['empty_start_blocked_frame']['quality']['candidates'][0]['corners'])


if __name__=='__main__':unittest.main()
