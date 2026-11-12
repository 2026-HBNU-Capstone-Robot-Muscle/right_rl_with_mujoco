"""Camera property restoration and fresh detection ranking; no real devices."""
import contextlib
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from camera_exposure_sweep import ExposureSession,exposure_values,frame_quality,quality_rank,main,sweep_worker

class Capture:
    def __init__(self):self.properties={1:.75,2:-4.};self.events=[];self.supported=True
    def get(self,key):return self.properties[key]
    def set(self,key,value):
        self.events.append((key,value))
        if not self.supported:return False
        self.properties[key]=float(value);return True

class Tests(unittest.TestCase):
    def test_setters_and_readbacks_restore_original_mode_last(self):
        cap=Capture();session=ExposureSession(cap,1,2)
        self.assertTrue(session.apply(-6)['supported'])
        self.assertEqual(cap.properties,{1:.25,2:-6.})
        restored=session.restore()
        self.assertTrue(restored['restored_original_readbacks'])
        self.assertEqual(cap.properties,{1:.75,2:-4.})
        self.assertEqual(cap.events[-2:],[(2,-4.),(1,.75)])

    def test_unsupported_properties_are_not_success_or_verified_restore(self):
        cap=Capture();cap.supported=False;session=ExposureSession(cap,1,2)
        self.assertFalse(session.apply(-6)['supported'])
        self.assertFalse(session.restore()['restored_original_readbacks'])

    def test_unknown_original_settings_cannot_be_changed(self):
        cap=Capture();cap.properties[1]=float('nan');session=ExposureSession(cap,1,2)
        self.assertFalse(session.apply(-6)['supported']);session.restore()
        self.assertEqual(cap.events,[])

    def test_decoded_pair_and_edge_size_required_without_inferred_ids(self):
        good=frame_quality([5,40],{'5':30,'40':80},.3)
        absent=frame_quality([5],{'5':80,'40':80},.01)
        small=frame_quality([5,40],{'5':18,'40':80},.01)
        duplicate=frame_quality([5,40,40],{'5':80,'40':80},.01)
        self.assertTrue(good['usable_pair'])
        self.assertTrue(all(not x['usable_pair'] for x in (absent,small,duplicate)))
        self.assertGreater(quality_rank(good),quality_rank(small))
        self.assertGreater(quality_rank(small),quality_rank(absent))

    def test_exposure_values_are_finite_bounded_and_no_brighter_than_known_start(self):
        for value in (-1.,-4.,-13.):
            options=exposure_values(value)
            self.assertEqual(len(options),len(set(options)))
            self.assertTrue(all(-13<=x<=value for x in options))
        self.assertEqual(exposure_values(float('nan')),[-3.,-4.,-5.,-6.,-7.])

    def test_default_entry_opens_nothing(self):
        with contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(main([]),0)
        self.assertIn('CHECK ONLY',output.getvalue())

    def run_worker(self,fault=None):
        import cv2 as cv
        cap=Capture();cap.properties={cv.CAP_PROP_AUTO_EXPOSURE:.75,cv.CAP_PROP_EXPOSURE:-4.}
        cap.release=lambda:setattr(cap,'released',True)
        reads=[];keys=[];sent=[]
        class Event:
            def __init__(self,flag=False):self.flag=flag
            def is_set(self):return self.flag
            def set(self):self.flag=True
            def wait(self,*args):return self.flag
        class Sender:
            def send(self,item):sent.append(item[0])
            def close(self):sent.append('closed')
        class Detector:
            stats={'min_edge_px_by_id':{'40':80.,'5':30.}}
            def detectMarkers(self,gray):return [],np.array([[40],[5]]),[]
            def draw(self,frame):pass
        def read(c,stop):
            reads.append(1)
            if fault=='read' and len(reads)==14:raise RuntimeError('injected camera read fault')
            return float(len(reads)),np.full((10,10,3),100,np.uint8)
        def key(delay):
            keys.append(1)
            return ord('q') if fault=='cancel' and len(keys)==14 else -1
        previous=Path.cwd()
        with tempfile.TemporaryDirectory() as folder,contextlib.ExitStack() as patches:
            for name in ('namedWindow','resizeWindow','imshow','destroyAllWindows'):
                patches.enter_context(patch.object(cv,name,return_value=None))
            patches.enter_context(patch.object(cv,'waitKey',side_effect=key))
            patches.enter_context(patch.object(cv,'getWindowProperty',return_value=1))
            patches.enter_context(patch('camera_stream.enumerate_names',return_value=['Streaming Webcam']))
            patches.enter_context(patch('camera_stream.read_frame',side_effect=read))
            patches.enter_context(patch('camera_720_check.open_hd',return_value=(cap,{},[])))
            patches.enter_context(patch('marker_detector.MarkerDetector',return_value=Detector()))
            try:
                with contextlib.redirect_stdout(io.StringIO()):
                    sweep_worker(Sender(),Event(),{'title':'FAKE'},folder=folder,start=Event(True))
                result=json.loads((Path(folder)/'worker_result.json').read_bytes())
                if not fault:self.assertTrue((Path(folder)/'best_frame.jpg').exists())
            finally:os.chdir(previous)
        return cap,result,sent

    def test_complete_worker_saves_raw_frames_and_restores_camera(self):
        import cv2 as cv
        cap,result,sent=self.run_worker()
        self.assertEqual(result['status'],'camera_exposure_diagnosis_complete')
        self.assertEqual(result['saved_raw_frames'],36)
        self.assertTrue(result['usable_pair_observed'])
        self.assertEqual(cap.properties,{cv.CAP_PROP_AUTO_EXPOSURE:.75,cv.CAP_PROP_EXPOSURE:-4.})
        self.assertTrue(result['restoration']['restored_original_readbacks'])
        self.assertTrue(cap.released);self.assertEqual(sent[-1],'closed')

    def test_worker_restores_after_real_control_flow_read_fault_or_q(self):
        import cv2 as cv
        for fault in ('read','cancel'):
            with self.subTest(fault=fault):
                cap,result,_=self.run_worker(fault)
                self.assertEqual(result['status'],'stopped')
                self.assertTrue(result['restoration']['changes_attempted'])
                self.assertTrue(result['restoration']['restored_original_readbacks'])
                self.assertEqual(cap.properties,{cv.CAP_PROP_AUTO_EXPOSURE:.75,cv.CAP_PROP_EXPOSURE:-4.})
                self.assertTrue(cap.released)

if __name__=='__main__':unittest.main()
