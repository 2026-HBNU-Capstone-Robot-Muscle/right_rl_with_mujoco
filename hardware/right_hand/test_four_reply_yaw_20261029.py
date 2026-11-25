import contextlib,io,json,sys,unittest
from types import SimpleNamespace
from unittest.mock import patch,Mock
import numpy as np,dynamixel_sdk as sdk
import fast_health_read4_20261029 as reader
import four_reply_yaw_plan_20261029 as plan
import four_reply_yaw_gate_20261029 as gate
import leap_four_reply_yaw_20261029 as entry
from leap_reference_speed_yaw_20261029 import ReferenceBus,ReferenceCapture
class Checks(unittest.TestCase):
    def fixture(self,fail_call=None,wrong_id=False,device_error=False):
        codec=sdk.PacketHandler(2.);sent=[];calls=0
        def tx(port,address,length,ids,count,fast):sent.append(list(ids));return 0
        def rx(port,fast):
            nonlocal calls
            calls+=1;ids=sent[-1]
            if calls==fail_call:return [0]*100,-3002
            total=8+len(ids)*87;raw=bytearray(b'\xff\xff\xfd\x00\xfe')+bytearray((total-7).to_bytes(2,'little'))+bytearray([0x55])
            for i,motor in enumerate(ids):
                data=bytearray(83);data[68:72]=(motor*100).to_bytes(4,'little')
                raw.extend([1 if device_error and i==0 else 0,motor+1 if wrong_id and i==0 else motor]);raw.extend(data)
                raw.extend(codec.updateCRC(0,list(raw),len(raw)).to_bytes(2,'little'))
            return list(raw),0
        port=SimpleNamespace(setPacketTimeoutMillis=Mock(),is_using=False)
        packet=SimpleNamespace(updateCRC=codec.updateCRC,syncReadTx=tx,rxPacket=rx)
        return SimpleNamespace(port=port,packet=packet,sdk=sdk,report={}),sent
    def test_every_ID_is_read_once_and_all_fields_return_only_when_complete(self):
        b,sent=self.fixture();state,comm,error,_=reader.block_once(b,64,83,[('position',132,4)])
        self.assertEqual(sent,[list(x) for x in reader.GROUPS]);np.testing.assert_array_equal(state['position'],np.arange(1,17)*100)
        self.assertEqual((comm,error),(0,0));self.assertEqual(b.report['fast_health_complete_reads'][-1]['groups'],4)
    def test_failed_third_group_returns_no_partial_snapshot(self):
        b,sent=self.fixture(fail_call=3);state,comm,error,detail=reader.block_once(b,64,83,[('position',132,4)])
        self.assertIsNone(state);self.assertEqual(comm,-3002);self.assertEqual(detail['completed_ids'],list(range(1,9)))
        self.assertEqual(detail['expected_packet_bytes'],356)
        state,comm,error,_=reader.block_once(b,64,83,[('position',132,4)])
        self.assertEqual(sent[3],list(range(1,5)));np.testing.assert_array_equal(state['position'],np.arange(1,17)*100)
    def test_CRC_valid_wrong_ID_is_rejected(self):
        b,_=self.fixture(wrong_id=True);state,comm,error,_=reader.block_once(b,64,83,[('position',132,4)])
        self.assertIsNone(state);self.assertEqual(comm,-3002)
    def test_individual_device_error_is_not_ignored(self):
        b,_=self.fixture(device_error=True);state,comm,error,_=reader.block_once(b,64,83,[('position',132,4)])
        self.assertIsNone(state);self.assertEqual(error,1)
    def test_shared_deadline_stops_before_later_groups(self):
        b,sent=self.fixture()
        with patch.object(reader.time,'perf_counter',side_effect=[0.,0.,.01,.21]):
            with self.assertRaises(RuntimeError):reader.block_once(b,64,83,[('position',132,4)])
        self.assertEqual(len(sent),1)
    def test_cancel_prevents_remaining_groups(self):
        b,sent=self.fixture();b.cancel_callback=Mock(side_effect=[None,RuntimeError('Q')])
        with self.assertRaises(RuntimeError):reader.block_once(b,64,83,[('position',132,4)])
        self.assertEqual(len(sent),1)
    def test_decoder_and_all_health_checks_are_original(self):
        from fast_health_read_20261029 import decode_status
        self.assertIs(reader.decode_status,decode_status);self.assertIs(entry.FourReplyBus.checked_sample,ReferenceBus.checked_sample)
        self.assertIs(entry.FourReplyBus._receive,ReferenceBus._receive)
    def test_goal_readback_and_OFF_configuration_are_not_redirected(self):
        b=object.__new__(entry.FourReplyBus);b.report={}
        for address,size in ((116,4),(64,1),(84,2)):
            with patch.object(ReferenceBus,'_block_once',return_value='ORIGINAL') as old,patch.object(entry,'fast4') as fast:
                self.assertEqual(b._block_once(address,size,[]),'ORIGINAL');old.assert_called_once();fast.assert_not_called()
    def test_actual_benchmark_and_current_model_review_validate(self):
        p=plan.FourReplyYawPlan(plan.REQUEST);r=gate.validate_review(p,plan.PROOF)
        self.assertEqual(r['four_reply_readonly_evidence']['OFF_camera_load_reads_each_method'],300)
        self.assertEqual(len(r['empty_recovery']['static_cases']),201);self.assertEqual(len(r['empty_recovery']['dynamic_cases']),9)
        self.assertEqual(r['active_health_read_policy']['maximum_reply_bytes'],356)
        self.assertEqual(r['hardware_tracking_stop_rad'],.18);self.assertEqual(r['active_motion_limit_s'],300)
    def test_trajectory_and_placement_are_the_same_completed_speed_route(self):
        class Reference(plan.Base):PLAN_VERSION=plan.FourReplyYawPlan.PLAN_VERSION
        a=Reference(plan.REQUEST);b=plan.FourReplyYawPlan(plan.REQUEST)
        aa=a.plan(a.source_position());bb=b.plan(b.source_position())
        self.assertEqual([p for p,q in aa],[p for p,q in bb]);np.testing.assert_array_equal([q for p,q in aa],[q for p,q in bb])
        self.assertIs(plan.FourReplyYawPlan.CAPTURE_PROTOCOL_TYPE,ReferenceCapture)
    def test_default_entry_opens_no_devices(self):
        import cv2
        with patch.object(sys,'argv',['leap_four_reply_yaw_20261029.py']),patch.object(cv2,'VideoCapture',side_effect=AssertionError('Camera opened')),patch.object(sdk,'PortHandler',side_effect=AssertionError('Motor opened')),contextlib.redirect_stdout(io.StringIO()):self.assertEqual(entry.main(),0)
if __name__=='__main__':unittest.main(verbosity=2)
