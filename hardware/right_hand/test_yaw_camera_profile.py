"""Fault-injected exposure selection/restoration, with no real devices."""
import contextlib
import json
from pathlib import Path
import queue
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2 as cv
import numpy as np
from fixed_exposure_camera import fixed_exposure_worker
from grasped_yaw_plan import GraspedYawPlan,GraspedYawCamera
from slow_grasp_demo_plan import SlowGraspDemoPlan
from empty_start_marker_check import make_empty_detector,inspect_empty_frame,EmptyStartEvidence

ROOT=Path(__file__).resolve().parent


def worker_case(mode='normal', exposure=-5.,confirm=False):
    events=[];messages=[];stop=threading.Event();counter=[0]
    props={cv.CAP_PROP_AUTO_EXPOSURE:-1.,cv.CAP_PROP_EXPOSURE:-4.};mismatched=[False]
    class Capture:
        def get(self,key):
            if key==cv.CAP_PROP_EXPOSURE and counter[0]>=7:
                if mode=='transient' and not mismatched[0]:mismatched[0]=True;return -1.
                if mode=='unavailable':return -1.
            return props[key]
        def set(self,key,value):
            events.append(('set',key,value))
            props[key]=-6. if mode=='unsupported' and key==cv.CAP_PROP_EXPOSURE and value==-5. else value
            return True
        def release(self):events.append(('release',))
    def read(*args):
        counter[0]+=1
        if counter[0]==7:
            if mode=='read':raise RuntimeError('camera read fault')
            if mode=='drift':props[cv.CAP_PROP_EXPOSURE]=-6.
            if mode=='normal':stop.set()
        return float(counter[0]),np.full((720,1280,3),100,np.uint8)
    with tempfile.TemporaryDirectory() as folder,contextlib.ExitStack() as stack:
        path=Path(folder)/'worker.json'
        stack.enter_context(patch('camera_stream.enumerate_names',return_value=['Streaming Webcam']))
        stack.enter_context(patch('camera_720_check.open_hd',return_value=(Capture(),{},[])))
        stack.enter_context(patch('camera_stream.read_frame',side_effect=read))
        stack.enter_context(patch('camera_stream.render_preview',return_value=('RECORD',mode in ('q','transient'))))
        for name in ('namedWindow','resizeWindow','destroyAllWindows'):
            stack.enter_context(patch.object(cv,name))
        fixed_exposure_worker(SimpleNamespace(send=messages.append,close=lambda:None),stop,
            dict(title='FAKE',updates=queue.Queue()),restoration_path=str(path),
            display_mode='record',exposure=exposure,confirm_property_reads=confirm)
        result=json.loads(path.read_bytes())
    return result,events,messages,props


class Tests(unittest.TestCase):
    def test_opted_in_worker_confirms_a_transient_read_without_more_setters(self):
        r,events,messages,props=worker_case('transient',confirm=True)
        self.assertTrue(r['property_read_confirmations'][0]['recovered'])
        self.assertEqual(r['property_read_confirmations'][0]['readbacks'],[-1.,-5.,-5.])
        self.assertEqual(sum(kind=='frame' for kind,_ in messages),1)
        self.assertEqual(sum(event[0]=='set' for event in events),4)
        self.assertTrue(r['restoration']['restored_original_readbacks'])

    def test_opted_in_worker_still_stops_on_persistent_unavailable_or_real_drift(self):
        for mode in ('unavailable','drift'):
            r,events,messages,props=worker_case(mode,confirm=True)
            with self.subTest(mode=mode):
                self.assertFalse(any(kind=='frame' for kind,_ in messages))
                self.assertTrue(any(kind=='error' for kind,_ in messages))
                self.assertFalse(r['property_read_confirmations'][0]['recovered'])

    def test_only_yaw_uses_bright_profile_and_worker_report_matches(self):
        self.assertIs(GraspedYawPlan.CAMERA_TYPE,GraspedYawCamera)
        self.assertEqual(GraspedYawCamera.EXPOSURE,-5.)
        self.assertEqual(SlowGraspDemoPlan.CAMERA_TYPE.EXPOSURE,-7.)
        report={};workers=[]
        def initialize(camera,folder,r,*,worker):workers.append(worker)
        with tempfile.TemporaryDirectory() as folder,patch('marker_trial_camera.make_empty_detector'), \
             patch('empty_trial_camera.EmptyTrialCamera.__init__',side_effect=initialize,autospec=True):
            GraspedYawCamera(Path(folder)/'camera',report)
        self.assertEqual(workers[0].keywords['exposure'],-5.)
        self.assertEqual(report['camera_requested_exposure'],-5.)
        self.assertFalse(report['marker_visibility_is_motion_gate'])

    def test_measured_minus5_stream_restores_original_before_release(self):
        r,events,messages,props=worker_case()
        self.assertEqual(r['requested_exposure'],-5.)
        self.assertEqual(r['last_stream_exposure_readback'],-5.)
        self.assertEqual(sum(kind=='frame' for kind,_ in messages),1)
        self.assertTrue(r['restoration']['restored_original_readbacks'])
        self.assertEqual(props[cv.CAP_PROP_EXPOSURE],-4.)
        self.assertLess(events.index(('set',cv.CAP_PROP_EXPOSURE,-4.)),events.index(('release',)))

    def test_record_preview_does_not_duplicate_parent_marker_detector(self):
        with patch('cube_marker_readiness.make_detector',side_effect=AssertionError('duplicate detector')):
            r,_,messages,_=worker_case()
        self.assertFalse(r['preview_marker_decoding'])
        self.assertTrue(any(kind=='frame' for kind,_ in messages))

    def test_q_read_failure_and_exposure_drift_restore_settings(self):
        for mode in ('q','read','drift'):
            with self.subTest(mode=mode):
                r,events,messages,props=worker_case(mode)
                self.assertTrue(r['restoration']['restored_original_readbacks'])
                self.assertEqual(props[cv.CAP_PROP_EXPOSURE],-4.)
                self.assertTrue(any(kind==('preview_stop' if mode=='q' else 'error') for kind,_ in messages))
                if mode!='q':self.assertFalse(any(kind=='frame' for kind,_ in messages))

    def test_unsupported_profile_sends_no_ready_frame(self):
        r,events,messages,_=worker_case('unsupported')
        self.assertFalse(any(kind in ('ready','frame') for kind,_ in messages))
        self.assertEqual(r['status'],'stopped')
        self.assertTrue(r['restoration']['restored_original_readbacks'])
        r,events,messages,_=worker_case(exposure=-8.)
        self.assertEqual(events,[])
        self.assertFalse(any(kind in ('devices','ready','frame') for kind,_ in messages))

    def test_selected_bright_empty_frames_do_not_confirm_glare_as_cube(self):
        folder=ROOT/'camera_exposure_sweep_20261023_220105_041489'
        result=json.loads((folder/'result.json').read_bytes());detector=make_empty_detector();evidence=EmptyStartEvidence()
        profile=next(p for p in result['profiles'] if p['name']=='manual_0')
        for row in profile['frames']:
            frame=cv.imdecode(np.frombuffer((folder/row['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
            quality=evidence.observe(row['camera_read_monotonic'],inspect_empty_frame(frame,detector))
            self.assertEqual(quality['confirmed_cube_ids'],[])

    def test_selected_bright_cube_frames_still_block_empty_start(self):
        folder=ROOT/'camera_exposure_sweep_20261023_220318_878174'
        result=json.loads((folder/'result.json').read_bytes());detector=make_empty_detector();evidence=EmptyStartEvidence()
        profile=next(p for p in result['profiles'] if p['name']=='manual_0')
        observed=[]
        for row in profile['frames']:
            frame=cv.imdecode(np.frombuffer((folder/row['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
            observed.extend(evidence.observe(row['camera_read_monotonic'],inspect_empty_frame(frame,detector))['confirmed_cube_ids'])
        self.assertIn(5,observed)


if __name__=='__main__':unittest.main()
