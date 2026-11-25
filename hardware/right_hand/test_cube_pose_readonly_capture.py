"""No real devices: bracket timing, OFF capability, and fixed-profile cleanup."""
import contextlib
import io
import json
import queue
from pathlib import Path
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from cube_pose_readonly_capture import collect_pairs,analyze_pairs,main,wait_for_capture_request
from fixed_exposure_camera import fixed_exposure_worker
from cube_marker_readiness import quality_from_ids,preview_with_markers,make_detector


def state(position=2048):
    return dict(torque=[0]*16,hardware_error=[0]*16,watchdog=[0]*16,
        position_ticks=[position]*16,voltage_v=[4.8]*16,temperature_c=[22]*16)


class Clock:
    def __init__(self):self.t=1.
    def now(self):return self.t
    def sleep(self,seconds):self.t+=seconds


class PairTests(unittest.TestCase):
    def test_space_waits_for_three_consecutive_current_marker_pairs(self):
        clock=Clock();keys=[' '];readable=iter((False,True,False,True,True,True));calls=[]
        camera=SimpleNamespace(read_packet=lambda:(clock.now(),b'raw'))
        def ready(packet):calls.append(packet);return next(readable)
        wait_for_capture_request(camera,lambda:keys.pop() if keys else None,ready=ready,
            now=clock.now,sleep=clock.sleep)
        self.assertEqual(len(calls),6)

    def test_readability_wait_is_finite_and_never_reads_a_motor(self):
        clock=Clock();keys=[' ']
        camera=SimpleNamespace(read_packet=lambda:(clock.now(),b'raw'))
        with self.assertRaisesRegex(RuntimeError,'within 30s'):
            wait_for_capture_request(camera,lambda:keys.pop() if keys else None,
                ready=lambda packet:False,now=clock.now,sleep=clock.sleep)

    def test_glare_missing_small_duplicate_markers_are_not_ready(self):
        for ids,edges in (([5],{'5':25}),([40],{'40':90}),
                          ([40,5],{'40':90,'5':23}),([40,40,5],{'40':90,'5':26})):
            self.assertFalse(quality_from_ids(ids,edges)['readable_pair'])
        self.assertTrue(quality_from_ids([40,5],{'40':90,'5':26})['readable_pair'])

    def test_live_marker_overlay_never_changes_raw_capture_pixels(self):
        image=np.full((720,1280,3),200,np.uint8);original=image.copy()
        display,streak,quality=preview_with_markers(image,make_detector(),0)
        self.assertTrue(np.array_equal(image,original));self.assertFalse(np.array_equal(display,original))
        self.assertEqual(streak,0);self.assertFalse(quality['readable_pair'])

    def test_zero_reads_never_claim_verified_off_endpoints(self):
        with tempfile.TemporaryDirectory() as temp:
            report={};analyze_pairs(Path(temp),report)
        self.assertEqual(report['completed_encoder_pair_count'],0)
        self.assertFalse(report['all_completed_pairs_have_off_endpoints'])
        self.assertEqual(report['usable_pair_count'],0)

    def test_one_space_between_camera_frames_is_latched(self):
        clock=Clock();keys=iter((' ',None,None));frames=iter((None,None,(1.02,b'raw')))
        camera=SimpleNamespace(read_packet=lambda:next(frames))
        wait_for_capture_request(camera,lambda:next(keys),now=clock.now,sleep=clock.sleep)
        self.assertAlmostEqual(clock.now(),1.02)

    def test_one_space_with_frozen_camera_times_out_without_requiring_another_space(self):
        clock=Clock();keys=[' ']
        with self.assertRaisesRegex(RuntimeError,'no fresh camera frame'):
            wait_for_capture_request(SimpleNamespace(read_packet=lambda:None),
                lambda:keys.pop() if keys else None,now=clock.now,sleep=clock.sleep)
        self.assertGreaterEqual(clock.now(),2.)

    def test_read_image_read_before_any_image_encoding(self):
        clock=Clock();events=[];report={}
        def read():events.append('read');clock.sleep(.031);return state()
        def frame():events.append('image');clock.sleep(.025);return clock.now(),b'raw'
        def save(name,image):events.append('encode');clock.sleep(.2)
        collect_pairs(read,frame,save,report,cancel=lambda:None,count=3,now=clock.now,sleep=clock.sleep)
        self.assertEqual(events,['read','image','read','encode']*3)
        self.assertEqual(len(report['pairs']),3)
        for row in report['pairs']:
            self.assertTrue(row['stationary_endpoints'])
            self.assertLess(row['before']['read_ended_monotonic'],row['camera_read_monotonic'])
            self.assertLessEqual(row['camera_read_monotonic'],row['after']['read_started_monotonic'])
            self.assertFalse(row['continuous_stationarity_verified'])
            self.assertAlmostEqual(row['frame_bracket_ms'],87.)

    def test_stale_and_future_frame_never_paired_with_fresh_readings(self):
        for kind in ('stale','future'):
            clock=Clock();report={};reads=[];saved=[]
            def read():reads.append(1);clock.sleep(.03);return state()
            def frame():
                return (clock.now()-1 if kind=='stale' else clock.now()+1),b'raw'
            with self.assertRaisesRegex(RuntimeError,'No fresh frame'):
                collect_pairs(read,frame,lambda *a:saved.append(a),report,cancel=lambda:None,
                    count=1,now=clock.now,sleep=clock.sleep)
            self.assertEqual(len(reads),1);self.assertFalse(saved)
            self.assertEqual(len(report['pairs']),1)

    def test_drift_is_retained_as_diagnostic_not_accepted_stationarity(self):
        clock=Clock();samples=iter((state(),state(2052)));report={};saved=[]
        def frame():clock.sleep(.01);return clock.now(),b'raw'
        collect_pairs(lambda:next(samples),frame,lambda *a:saved.append(a),report,
            cancel=lambda:None,count=1,now=clock.now,sleep=clock.sleep)
        self.assertEqual(report['pairs'][0]['maximum_drift_ticks'],4)
        self.assertEqual(report['pairs'][0]['status'],'endpoint_drift')
        self.assertFalse(report['pairs'][0]['stationary_endpoints']);self.assertEqual(len(saved),1)

    def test_on_hardware_error_watchdog_are_saved_and_not_reset(self):
        for field,value in (('torque',1),('hardware_error',4),('watchdog',255)):
            report={};sample=state();sample[field][0]=value;called=[]
            with self.assertRaises(RuntimeError):
                collect_pairs(lambda:sample,lambda:called.append('frame'),lambda *a:called.append('save'),
                    report,cancel=lambda:None,count=1)
            self.assertEqual(called,[])
            self.assertEqual(report['pairs'][0]['before']['snapshot'][field][0],value)

    def test_slow_read_stops_before_frame_and_has_partial_read_record(self):
        clock=Clock();report={};frames=[]
        def read():clock.sleep(.201);return state()
        with self.assertRaisesRegex(RuntimeError,'200ms'):
            collect_pairs(read,lambda:frames.append(1),lambda *a:None,report,cancel=lambda:None,
                count=1,now=clock.now,sleep=clock.sleep)
        self.assertEqual(frames,[]);self.assertAlmostEqual(report['pairs'][0]['before']['read_ms'],201.)

    def test_q_before_first_read_has_no_device_operation(self):
        calls=[]
        def cancel():raise KeyboardInterrupt('Q')
        with self.assertRaises(KeyboardInterrupt):
            collect_pairs(lambda:calls.append('read'),lambda:calls.append('image'),
                lambda *a:calls.append('save'),{},cancel=cancel,count=1)
        self.assertEqual(calls,[])

    def test_default_check_only_never_constructs_bus_or_camera(self):
        with patch('cube_pose_readonly_capture.ReadonlyBus',side_effect=AssertionError('No device')):
            with patch('camera_stream.CameraStream',side_effect=AssertionError('No device')):
                with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(main([]),0)

    def test_saved_marker_frame_requires_matching_stable_pair_record(self):
        import cv2 as cv
        # Use synthetic, front-facing markers, not a real device or old q.
        image=np.full((720,1280,3),255,np.uint8)
        dictionary=cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        for ident,x in ((40,180),(5,640)):
            marker=cv.aruco.generateImageMarker(dictionary,ident,140)
            image[200:340,x:x+140]=cv.cvtColor(marker,cv.COLOR_GRAY2BGR)
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);ok,encoded=cv.imencode('.png',image);self.assertTrue(ok)
            (folder/'pair.png').write_bytes(encoded.tobytes())
            rows=[dict(index=i,image='pair.png',status=status)
                  for i,status in enumerate(('paired','endpoint_drift','started'))]
            report=dict(pairs=rows);analyze_pairs(folder,report)
            self.assertEqual(report['usable_pair_indices'],[0])
            self.assertFalse(report['geometry_calibration_verified'])
            self.assertFalse(report['old_encoder_readings_reused'])


class WorkerTests(unittest.TestCase):
    def run_worker(self,mode):
        import cv2 as cv
        events=[];messages=[];stop=threading.Event();props={cv.CAP_PROP_AUTO_EXPOSURE:-1.,cv.CAP_PROP_EXPOSURE:-5.}
        class Cap:
            def get(self,key):return props[key]
            def set(self,key,value):
                events.append(('set',key,value));props[key]=(-6. if mode=='unsupported' and value==-7 else value);return True
            def release(self):events.append(('release',))
        cap=Cap();frame=np.zeros((720,1280,3),np.uint8);counter=[0]
        def read(*args):
            counter[0]+=1
            if counter[0]==7:
                if mode=='read_failure':raise RuntimeError('Fake camera failure')
                if mode=='normal':stop.set()
                if mode=='exposure_drift':props[cv.CAP_PROP_EXPOSURE]=-5.
            return float(counter[0]),frame
        def render(*args):return 'READ ONLY',mode=='q'
        def send(item):messages.append(item)
        def closed():events.append(('pipe_closed',))
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'restoration.json'
            with patch('camera_stream.enumerate_names',return_value=['Streaming Webcam']), \
                 patch('camera_720_check.open_hd',return_value=(cap,dict(actual_size=[1280,720]),[])), \
                 patch('camera_stream.read_frame',side_effect=read), \
                 patch('camera_stream.render_preview',side_effect=render), \
                 patch.object(cv,'namedWindow'),patch.object(cv,'resizeWindow'),patch.object(cv,'destroyAllWindows'):
                fixed_exposure_worker(SimpleNamespace(send=send,close=closed),stop,
                    dict(title='FAKE',updates=queue.Queue()),restoration_path=str(path))
            record=json.loads(path.read_bytes())
        return record,events,messages,props

    def test_normal_close_restores_originals_even_when_auto_getter_is_minus_one(self):
        import cv2 as cv
        record,events,messages,props=self.run_worker('normal')
        self.assertEqual(record['status'],'closed')
        self.assertTrue(record['restoration']['restored_original_readbacks'])
        self.assertFalse(record['auto_mode_meaning_verified'])
        self.assertEqual(props[cv.CAP_PROP_AUTO_EXPOSURE],-1.)
        self.assertEqual(props[cv.CAP_PROP_EXPOSURE],-5.)
        self.assertEqual(sum(k=='frame' for k,v in messages),1)
        self.assertLess(events.index(('set',cv.CAP_PROP_AUTO_EXPOSURE,-1.)),events.index(('release',)))

    def test_q_restores_properties_and_notifies_parent(self):
        record,events,messages,props=self.run_worker('q')
        self.assertEqual(record['status'],'user_stop')
        self.assertTrue(record['restoration']['restored_original_readbacks'])
        self.assertTrue(any(k=='preview_stop' for k,v in messages))

    def test_read_fault_and_exposure_drift_restore_and_stop(self):
        for mode in ('read_failure','exposure_drift'):
            record,events,messages,props=self.run_worker(mode)
            self.assertEqual(record['status'],'stopped')
            self.assertTrue(record['restoration']['restored_original_readbacks'])
            self.assertFalse(any(k=='frame' for k,v in messages))
            self.assertTrue(any(k=='error' for k,v in messages))

    def test_unsupported_exposure_never_sends_ready_or_frame(self):
        record,events,messages,props=self.run_worker('unsupported')
        self.assertEqual(record['status'],'stopped')
        self.assertTrue(record['restoration']['restored_original_readbacks'])
        self.assertFalse(any(k in ('ready','frame') for k,v in messages))


if __name__=='__main__':unittest.main()
