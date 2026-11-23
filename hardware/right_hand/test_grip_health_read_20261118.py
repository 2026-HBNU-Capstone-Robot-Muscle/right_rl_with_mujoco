"""Use the real SDK parser against fragmented, corrupt, missing and error replies."""
import time,unittest
from types import SimpleNamespace
from unittest.mock import patch
import dynamixel_sdk as sdk
from grip_health_read_20261118 import block_once,install

class Port:
    def __init__(self,codec,*,bad_crc=None,status_error=None,missing=None,chunk=7):
        self.codec=codec;self.bad_crc=bad_crc;self.status_error=status_error;self.missing=missing
        self.chunk=chunk;self.buffer=bytearray();self.is_using=False;self.deadline=0.;self.transmits=[]
        self.ser=SimpleNamespace(timeout=0)
    def clearPort(self):self.buffer.clear()
    def setPacketTimeout(self,length):self.setPacketTimeoutMillis(40.)
    def setPacketTimeoutMillis(self,ms):self.deadline=time.perf_counter()+ms/1000
    def isPacketTimeout(self):return time.perf_counter()>=self.deadline
    def readPort(self,n):
        n=min(n,self.chunk,len(self.buffer));data=bytes(self.buffer[:n]);del self.buffer[:n];return data
    def writePort(self,request):
        self.transmits.append(list(request));assert request[7]==0x82
        for motor in request[12:-2]:
            if motor==self.missing:continue
            data=[0]*83;data[1:5]=[255,255,253,0];data[68:72]=motor.to_bytes(4,'little')
            frame=[255,255,253,0,motor,87,0,0x55,128 if motor==self.status_error else 0]+data+[0,0]
            self.codec.addStuffing(frame)
            crc=self.codec.updateCRC(0,frame,len(frame)-2);frame[-2:]=crc.to_bytes(2,'little')
            if motor==self.bad_crc:frame[-1]^=1
            self.buffer.extend(frame)
        return len(request)

def fixture(**kwargs):
    packet=sdk.PacketHandler(2.);port=Port(packet,**kwargs)
    return SimpleNamespace(packet=packet,port=port,sdk=sdk,report={},cancel_callback=lambda:None)

class ReaderTests(unittest.TestCase):
    def test_real_sdk_fragmented_stuffed_packets_all16_and_only_paired_READs(self):
        bus=fixture(chunk=2);install(bus)
        state,comm,error,_=block_once(bus,64,83,[('position',132,4)])
        self.assertEqual((comm,error),(0,0));self.assertEqual(state['position'].tolist(),list(range(1,17)))
        self.assertEqual([p[12:-2] for p in bus.port.transmits],[[n,n+1] for n in range(1,17,2)])
        self.assertEqual(bus.port.ser.timeout,.001)
        self.assertNotIn('failed_raw_packets',bus.report)

    def test_real_crc_failure_rejects_whole_snapshot_and_records_raw(self):
        bus=fixture(bad_crc=6);install(bus)
        state,comm,error,detail=block_once(bus,64,83,[('position',132,4)])
        self.assertIsNone(state);self.assertEqual(comm,sdk.COMM_RX_CORRUPT)
        self.assertEqual(detail['completed_ids'],[1,2,3,4,5])
        self.assertEqual(detail['first_incomplete_id'],6)
        self.assertTrue(bus.report['failed_raw_packets'][0]['length_complete'])
        self.assertFalse(bus.report['failed_raw_packets'][0]['crc_matches'])

    def test_status_error_is_preserved_and_not_retried(self):
        bus=fixture(status_error=3);install(bus)
        state,comm,error,detail=block_once(bus,64,83,[('position',132,4)])
        self.assertIsNone(state);self.assertEqual((comm,error),(0,128));self.assertEqual(detail['device_error_id'],3)
        self.assertEqual(len(bus.port.transmits),2)

    def test_missing_motor_never_returns_partial_snapshot(self):
        bus=fixture(missing=8)
        state,comm,_,detail=block_once(bus,64,83,[('position',132,4)])
        self.assertIsNone(state);self.assertEqual(comm,sdk.COMM_RX_TIMEOUT)
        self.assertEqual(detail['completed_ids'],list(range(1,8)))
        self.assertFalse(bus.port.is_using)

    def test_expired_shared_deadline_sends_no_requests(self):
        bus=fixture();bus._horizontal_sample_deadline=time.perf_counter()-.01
        with self.assertRaisesRegex(RuntimeError,'Shared200ms'):block_once(bus,64,83,[('position',132,4)])
        self.assertEqual(bus.port.transmits,[])

    def test_data_arriving_after_shared_deadline_is_not_accepted(self):
        bus=fixture();bus._horizontal_sample_deadline=time.perf_counter()+.001
        original=bus.packet.readRx
        def delayed(*args):
            value=original(*args);time.sleep(.005);return value
        bus.packet.readRx=delayed
        with self.assertRaisesRegex(RuntimeError,'shared200ms'):block_once(bus,64,83,[('position',132,4)])

    def test_raw_error_trace_is_bounded_and_keeps_parser_result(self):
        bus=fixture()
        bus.packet.rxPacket=lambda *_:([255,255],sdk.COMM_RX_CORRUPT)
        install(bus)
        for _ in range(70):self.assertEqual(bus.packet.rxPacket(bus.port,False),([255,255],sdk.COMM_RX_CORRUPT))
        self.assertEqual(len(bus.report['failed_raw_packets']),64)
        self.assertFalse(bus.report['failed_raw_packets'][0]['length_complete'])

    def test_invalid_field_scope_sends_no_request(self):
        bus=fixture()
        with self.assertRaises(ValueError):block_once(bus,64,83,[('outside',147,1)])
        self.assertEqual(bus.port.transmits,[])

    def test_host_configuration_failure_closes_port_without_motor_writes(self):
        import leap_measured_grip_hold_20261117 as pose
        calls=[]
        port=SimpleNamespace(is_using=False,openPort=lambda:True,setBaudRate=lambda _:True,
                             closePort=lambda:calls.append('closed'))
        adapter=SimpleNamespace(vid=0x0403,pid=0x6014,serial_number='FTBENPSAA',device='FAKE')
        with patch('serial.tools.list_ports.comports',return_value=[adapter]),patch('dynamixel_sdk.PortHandler',return_value=port),patch('grip_health_read_20261118.install',side_effect=RuntimeError('host timeout rejected')):
            with self.assertRaisesRegex(RuntimeError,'host timeout rejected'):pose.PoseBus({},lambda:None)
        self.assertEqual(calls,['closed'])

if __name__=='__main__':unittest.main()
