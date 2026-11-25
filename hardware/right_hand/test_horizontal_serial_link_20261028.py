"""No-device faults for host speed selection and complete grouped snapshots."""
import copy,json,unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
import horizontal_serial_link_20261028 as link
from horizontal_sync_receive_window_20261028 import block_once

class HostLinkTests(unittest.TestCase):
    def test_real_evidence_binds_all16_and_chunked_comparison(self):
        policy=link.metadata();self.assertEqual(policy['baud'],1000000)
        self.assertEqual(policy['motor_ids'],list(range(1,17)));self.assertEqual(policy['full_health_reply_group_size'],1)
    def test_host_selection_does_not_write_motor_registers(self):
        bus=SimpleNamespace(port=SimpleNamespace(getBaudRate=lambda:3000000,setBaudRate=Mock(return_value=True)),
            report={},write=Mock())
        link.prepare_host(bus);bus.port.setBaudRate.assert_called_once_with(1000000);bus.write.assert_not_called()
    def test_already_matched_host_is_not_reconfigured(self):
        bus=SimpleNamespace(port=SimpleNamespace(getBaudRate=lambda:1000000,setBaudRate=Mock()),report={})
        link.prepare_host(bus);bus.port.setBaudRate.assert_not_called()
    def test_failed_host_selection_stops(self):
        bus=SimpleNamespace(port=SimpleNamespace(getBaudRate=lambda:3000000,setBaudRate=lambda n:False),report={})
        with self.assertRaises(RuntimeError):link.prepare_host(bus)
    def test_preflight_checks_all16_baud_registers_before_motion(self):
        import leap_horizontal_quarter_20261027 as live
        bus=live.HorizontalQuarterBus.__new__(live.HorizontalQuarterBus);bus.read=Mock(return_value=3)
        with patch.object(live,'prepare_host',return_value={'expected_baud_register':3}),patch.object(live.NewPairBus,'preflight',return_value='OFF'):
            self.assertEqual(bus.preflight(),'OFF');self.assertEqual(bus.read.call_count,16)
            bus.read.assert_any_call(16,8)
    def test_one_motor_at_wrong_baud_register_is_rejected(self):
        import leap_horizontal_quarter_20261027 as live
        bus=live.HorizontalQuarterBus.__new__(live.HorizontalQuarterBus);bus.read=Mock(side_effect=[3,5])
        with patch.object(live,'prepare_host',return_value={'expected_baud_register':3}),patch.object(live.NewPairBus,'preflight',return_value='OFF'):
            with self.assertRaisesRegex(RuntimeError,'ID2'):bus.preflight()
    def test_changed_or_unverified_config_is_rejected(self):
        original=json.loads(link.CONFIG.read_bytes())
        with TemporaryDirectory() as temporary:
            for updates in ({'baud':3000000},{'status':'started'},{'evidence_sha256':'0'*64},{'motor_ids':[1]}):
                value=dict(original,**updates);path=Path(temporary)/'link.json';path.write_text(json.dumps(value),encoding='utf-8')
                with patch.object(link,'CONFIG',path):
                    with self.assertRaises(RuntimeError):link.metadata()
    def test_operator_proof_cannot_change_the_verified_host_speed(self):
        import horizontal_quarter_plan_20261027 as p
        from horizontal_quarter_gate_20261027 import validate_review
        value=json.loads(p.PROOF.read_bytes());value['verified_serial_link_policy']['baud']=3000000
        with TemporaryDirectory() as temporary:
            path=Path(temporary)/'bad.json';path.write_text(json.dumps(value),encoding='utf-8')
            with self.assertRaisesRegex(RuntimeError,'mismatch'):validate_review(p.HorizontalQuarterPlan(p.REQUEST),path)

class Chunk:
    def __init__(self,owner,length,fail=False):self.owner=owner;self.length=length;self.fail=fail;self.data_dict={};self.cleared=False
    def addParam(self,motor):self.data_dict[motor]=[];return True
    def txPacket(self):self.owner.requests.append(list(self.data_dict));return 0
    def rxPacket(self):
        for motor in self.data_dict:self.data_dict[motor]=[] if self.fail else [0]*self.length
        return -3002 if self.fail else 0
    def isAvailable(self,motor,address,size):return len(self.data_dict[motor])==self.length
    def getData(self,motor,address,size):return motor*10000+address
    def clearParam(self):self.data_dict.clear();self.cleared=True

class BurstTests(unittest.TestCase):
    def make(self,fail_second=False):
        owner=SimpleNamespace(requests=[],groups=[])
        def factory(port,packet,address,length):
            g=Chunk(owner,length,fail=fail_second and len(owner.groups)==1);owner.groups.append(g);return g
        bus=SimpleNamespace(sdk=SimpleNamespace(COMM_SUCCESS=0,GroupSyncRead=factory),
            port=SimpleNamespace(setPacketTimeoutMillis=Mock()),packet=object(),report={},write=Mock())
        return bus,owner
    def test_full_health_has_two_groups_and_keeps_every_requested_field(self):
        bus,owner=self.make();fields=[('position',132,4),('voltage',144,2),('p_gain',84,2)]
        result,comm,_,_=block_once(bus,64,83,fields)
        self.assertEqual(owner.requests,[[i] for i in range(1,17)]);self.assertEqual(comm,0)
        for key,address,size in fields:self.assertEqual(result[key].tolist(),[i*10000+address for i in range(1,17)])
        self.assertTrue(all(g.cleared for g in owner.groups));bus.write.assert_not_called()
    def test_failed_second_half_returns_no_partial_snapshot(self):
        bus,owner=self.make(fail_second=True)
        result,comm,_,detail=block_once(bus,64,83,[('position',132,4)])
        self.assertIsNone(result);self.assertEqual(comm,-3002);self.assertEqual(detail['completed_ids'],[1])
        self.assertEqual(detail['first_incomplete_id'],2);bus.write.assert_not_called()
        # Another attempt requests the first half again; no cached old half.
        result,comm,_,_=block_once(bus,64,83,[('position',132,4)])
        self.assertEqual(comm,0);self.assertEqual(owner.requests[2],[1])
    def test_small_torque_off_reply_remains_a_single16_id_request(self):
        bus,owner=self.make();block_once(bus,64,1,[('torque',64,1)])
        self.assertEqual(owner.requests,[list(range(1,17))])
    def test_common_deadline_is_not_restarted_for_the_second_half(self):
        bus,owner=self.make();bus._horizontal_sample_deadline=10.2
        clock=iter([10.,10.,10.01,10.02,10.1,10.11,10.15])
        with patch('horizontal_sync_receive_window_20261028.time.monotonic',side_effect=lambda:next(clock,10.15)):
            block_once(bus,64,83,[('position',132,4)])
        windows=[args.args[0] for args in bus.port.setPacketTimeoutMillis.call_args_list]
        self.assertTrue(all(0<w<=80 for w in windows));self.assertEqual(len(windows),16)
    def test_sixteen_slow_individual_replies_cannot_extend_the_whole_budget(self):
        bus,owner=self.make();clock=[10.];factory=bus.sdk.GroupSyncRead
        def timed(*args):
            group=factory(*args);receive=group.rxPacket
            def delayed():clock[0]+=.03;return receive()
            group.rxPacket=delayed;return group
        bus.sdk.GroupSyncRead=timed
        with patch('horizontal_sync_receive_window_20261028.time.monotonic',side_effect=lambda:clock[0]):
            with self.assertRaisesRegex(RuntimeError,'deadline'):block_once(bus,64,83,[('position',132,4)])
        self.assertLess(len(owner.requests),16);bus.write.assert_not_called()

if __name__=='__main__':unittest.main()
