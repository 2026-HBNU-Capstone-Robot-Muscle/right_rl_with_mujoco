import copy
import unittest
from types import SimpleNamespace
from leap_pose_pair_readonly import validate_pair, capture_pair, summarize_tip_direction, direction_protocol
from hand_readonly_check import inspect_bus


def snapshot():
    return dict(status='read_complete_not_motion_approval',motors=[dict(id=i,torque_enabled=0,
                torque_enabled_end=0,hardware_error=0,position_ticks_signed=2048) for i in range(1,17)])


class PairTests(unittest.TestCase):
    def test_direction_observation_is_not_calibration(self):
        baseline=dict(status='photo_and_readings_saved',snapshots=[snapshot(),snapshot()])
        bent=copy.deepcopy(baseline)
        for row in bent['snapshots']:row['motors'][11]['position_ticks_signed']+=128
        result=summarize_tip_direction([baseline,bent])
        self.assertEqual(result['delta_degrees_by_motor']['12'],11.25)
        self.assertEqual(result['status'],'ready_for_visual_direction_review')
        self.assertFalse(result['physical_direction_verified'])
        self.assertFalse(result['calibration_selected'])
        for row in bent['snapshots']:row['motors'][8]['position_ticks_signed']+=64
        self.assertEqual(summarize_tip_direction([baseline,bent])['status'],'multiple_changes_or_small_motion_needs_review')
        self.assertEqual(summarize_tip_direction([baseline,baseline])['status'],'multiple_changes_or_small_motion_needs_review')
        bent['snapshots'][1]['motors'][11]['position_ticks_signed']+=10
        with self.assertRaises(RuntimeError):summarize_tip_direction([baseline,bent])

    def test_unknown_tip_id_is_observation_not_rejected_mapping(self):
        baseline=dict(status='photo_and_readings_saved',snapshots=[snapshot(),snapshot()])
        bent=copy.deepcopy(baseline)
        for row in bent['snapshots']:
            row['motors'][8]['position_ticks_signed']+=108
            row['motors'][10]['position_ticks_signed']+=23
            row['motors'][11]['position_ticks_signed']-=19
        result=summarize_tip_direction([baseline,bent])
        self.assertEqual(result['largest_changed_motor_id'],9)
        self.assertEqual(result['status'],'ready_for_visual_direction_review')
        self.assertIsNone(result['assumed_anatomical_motor_id'])
        self.assertFalse(result['calibration_selected'])
        self.assertFalse(result['physical_direction_verified'])

    def test_pip_protocol_is_distinct_and_has_separate_result(self):
        tip=direction_protocol('middle_tip_direction');pip=direction_protocol('middle_pip_order')
        self.assertIn('SECOND',pip['instructions'][0])
        self.assertIn('TOGETHER',pip['instructions'][1])
        self.assertNotEqual(tip['pointer'],pip['pointer'])
        self.assertIsNone(direction_protocol('current'))

    def test_burst_chooses_best_fresh_frame_between_reads(self):
        clock=[10.];events=[];saved=[];candidates=[]
        def inspect(row):events.append('read');row.update(snapshot())
        # First stale frame and a repeated packet must never be ranked/saved.
        packets=iter([(9.,99),(10.1,1),(10.1,100),(10.3,2),(10.4,8),(10.5,3),(10.6,4)])
        camera=SimpleNamespace(read_packet=lambda:next(packets,None))
        def sleep(_):clock[0]=round(clock[0]+.1,3)
        # Make the first fresh packet's timestamp available on its read.
        def read_packet():
            packet=next(packets,None)
            if packet is not None:clock[0]=max(clock[0],packet[0])
            return packet
        camera.read_packet=read_packet
        report={}
        self.assertTrue(capture_pair(inspect,camera,report,lambda im:(saved.append(im),events.append('photo')),
            now=lambda:clock[0],sleep=sleep,burst_seconds=.5,score_image=float,
            save_candidate=lambda i,im:candidates.append(im)))
        self.assertEqual(saved,[8]);self.assertEqual(candidates,[1,2,8,3,4])
        self.assertEqual(events,['read','photo','read'])
        self.assertEqual(report['photo_burst']['selected_index'],2)
        self.assertEqual(report['camera_read_monotonic'],10.4)

    def test_burst_camera_loss_does_not_accept_one_frame(self):
        clock=[10.];calls=[];packets=iter([(10.1,1)])
        def inspect(row):calls.append(1);row.update(snapshot())
        def read_packet():
            packet=next(packets,None)
            if packet:clock[0]=packet[0]
            return packet
        def sleep(_):clock[0]+=.1
        with self.assertRaisesRegex(RuntimeError,'Fewer than 5'):
            capture_pair(inspect,SimpleNamespace(read_packet=read_packet),{},
                lambda _:self.fail('accepted incomplete burst'),now=lambda:clock[0],sleep=sleep,
                burst_seconds=.5,score_image=float)
        self.assertEqual(len(calls),1)

    def test_drift_and_incomplete_and_torque(self):
        a=snapshot();b=copy.deepcopy(a)
        self.assertTrue(validate_pair(a,b)['stationary_endpoints'])
        b['motors'][11]['position_ticks_signed']+=4
        self.assertFalse(validate_pair(a,b)['stationary_endpoints'])
        b['motors'][11]['torque_enabled_end']=1
        with self.assertRaises(RuntimeError): validate_pair(a,b)
        b=snapshot();b['motors'].pop()
        with self.assertRaises(RuntimeError): validate_pair(a,b)

    def test_photo_is_new_and_between_reads(self):
        clock=[10.];events=[];saved=[]
        def inspect(row): events.append('read');row.update(snapshot());clock[0]+=.5
        packets=iter([(9.,'old'),(10.6,'new')])
        camera=SimpleNamespace(read_packet=lambda: next(packets))
        def sleep(_): clock[0]+=.2
        report={}
        self.assertTrue(capture_pair(inspect,camera,report,lambda im:(saved.append(im),events.append('photo')),
                                    now=lambda:clock[0],sleep=sleep))
        self.assertEqual(saved,['new']);self.assertEqual(events,['read','photo','read'])

    def test_camera_loss_no_second_read_or_old_photo(self):
        clock=[0.];calls=[]
        def inspect(row):calls.append(1);row.update(snapshot())
        def sleep(_):clock[0]+=.5
        with self.assertRaisesRegex(RuntimeError,'No fresh'):
            capture_pair(inspect,SimpleNamespace(read_packet=lambda:None),{},lambda _:self.fail('saved stale image'),
                         now=lambda:clock[0],sleep=sleep)
        self.assertEqual(len(calls),1)

    def test_invalid_initial_read_never_captures(self):
        for failure in ('torque', 'partial', 'communication'):
            def inspect(row):
                if failure == 'communication': raise RuntimeError('read timeout')
                row.update(snapshot())
                if failure == 'torque': row['motors'][0]['torque_enabled']=1
                if failure == 'partial': row['motors'].pop()
            camera=SimpleNamespace(read_packet=lambda:self.fail('camera read after failed inspection'))
            with self.assertRaises(RuntimeError):
                capture_pair(inspect,camera,{},lambda _:self.fail('photo after failed inspection'))

    def test_existing_inspector_has_only_read_capability_and_closes(self):
        calls=[];closed=[]
        class Port:
            def openPort(self):return True
            def setBaudRate(self,b):return True
            def closePort(self):closed.append(True)
        class Packet:
            def ping(self,p,i):calls.append(('ping',i));return 1240,0,0
            def read1ByteTxRx(self,p,i,a):calls.append(('read1',i,a));return 0,0,0
            def read2ByteTxRx(self,p,i,a):calls.append(('read2',i,a));return 48,0,0
            def read4ByteTxRx(self,p,i,a):calls.append(('read4',i,a));return (2048 if a==132 else 0),0,0
            def __getattr__(self,name):raise AssertionError('Unexpected SDK capability: '+name)
        sdk=SimpleNamespace(PortHandler=lambda _:Port(),PacketHandler=lambda _:Packet(),COMM_SUCCESS=0)
        report={'motors':[]};inspect_bus(sdk,'FAKE',report)
        self.assertEqual(len(report['motors']),16);self.assertEqual(closed,[True])
        self.assertTrue(all(c[0] in ('ping','read1','read2','read4') for c in calls))


if __name__=='__main__':unittest.main()
