"""All-ID diagnosis and receive trace with fake SDK/clock only."""
import contextlib
import io
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import bus_readonly_diagnose as d


class Clock:
    def __init__(self):self.t=0.
    def now(self):return self.t
    def sleep(self,s):self.t+=s


def fixture(*,missing=(),unsupported=(),bad_read=None,stream_comm=0,stream_field=None):
    clock=Clock();events=[];report={}
    class Packet:
        def ping(self,port,ident):
            events.append(('ping',ident));clock.sleep(.01)
            return (0,-3001,0) if ident in missing else (1220 if ident in unsupported else 1240,0,0)
        def read1ByteTxRx(self,port,ident,address):return self.read(ident,address)
        def read2ByteTxRx(self,port,ident,address):return self.read(ident,address)
        def read(self,ident,address):
            events.append(('read',ident,address));clock.sleep(.01)
            if bad_read and (ident,address)==bad_read[:2]:return bad_read[2:]
            return {64:0,70:0,98:0,144:48,146:22}[address],0,0
        def rxPacket(self,port,fast):return [255,255],-3002
        def __getattr__(self,name):raise AssertionError('Forbidden packet API: '+name)
    class Group:
        def addParam(self,ident):return True
        def txRxPacket(self):events.append(('sync_read',64,83));clock.sleep(.031);return stream_comm
        def isAvailable(self,*args):return True
        def getData(self,ident,address,size):
            value={64:0,70:0,98:0,132:2048,144:48,146:22}[address]
            return stream_field[2] if stream_field and (ident,address)==stream_field[:2] else value
    port=SimpleNamespace(ser=None,openPort=lambda:events.append(('open',)) or True,
        setBaudRate=lambda value:events.append(('host_baud',value)) or True,
        closePort=lambda:events.append(('close',)))
    def group(p,k,address,length):
        if (address,length)!=(64,83):raise AssertionError('Unexpected READ address')
        return Group()
    sdk=SimpleNamespace(COMM_SUCCESS=0,PortHandler=lambda device:port,
        PacketHandler=lambda version:Packet(),GroupSyncRead=group)
    bus=d.DiagnosticBus(sdk,'FAKE',report)
    return bus,report,clock,events


class Tests(unittest.TestCase):
    def setUp(self):
        self.output=patch('sys.stdout',new_callable=io.StringIO);self.output.start();self.addCleanup(self.output.stop)

    def run_diagnostic(self,bus,report,clock,cancel=lambda:None):
        return d.diagnose(bus,report,cancel=cancel,now=clock.now,sleep=clock.sleep)

    def test_all_ids_are_pinged_three_times_then_same_83_byte_stream(self):
        bus,r,clock,events=fixture();bus.open();passed=self.run_diagnostic(bus,r,clock);bus.close()
        self.assertTrue(passed);self.assertEqual(r['status'],'camera_closed_all_16_read_only_check_complete')
        self.assertEqual(r['responding_ids'],list(range(1,17)));self.assertEqual(len(r['motor_scans']),48)
        self.assertEqual(sum(e[0]=='ping' for e in events),48)
        self.assertEqual(sum(e[0]=='read' for e in events),240)
        self.assertGreaterEqual(len(r['stream_samples']),99)
        self.assertTrue(r['all_16_torque_OFF_verified']);self.assertTrue(r['scan_timing_within_200ms'])
        self.assertEqual(r['servo_write_commands_sent'],0);self.assertFalse(r['camera_opened'])
        self.assertTrue(all(e[0] in ('open','host_baud','ping','read','sync_read','close') for e in events))

    def test_missing_ID1_does_not_hide_other_fifteen_and_does_not_stream(self):
        bus,r,clock,events=fixture(missing=(1,));self.assertFalse(self.run_diagnostic(bus,r,clock))
        self.assertEqual(r['responding_ids'],list(range(2,17)))
        self.assertEqual(r['ping_success_count_by_id']['1'],0)
        self.assertEqual(sum(e[0]=='ping' for e in events),48)
        self.assertFalse(any(e[0]=='sync_read' for e in events));self.assertFalse(r['all_16_torque_OFF_verified'])

    def test_no_responses_do_not_get_interpreted_as_torque_off(self):
        bus,r,clock,events=fixture(missing=range(1,17));self.assertFalse(self.run_diagnostic(bus,r,clock))
        self.assertEqual(r['responding_ids'],[]);self.assertFalse(r['all_16_torque_OFF_verified'])
        self.assertTrue(all(e[0]=='ping' for e in events))

    def test_unsupported_model_is_not_decoded_as_XC330_M288(self):
        bus,r,clock,events=fixture(unsupported=(7,));self.assertFalse(self.run_diagnostic(bus,r,clock))
        self.assertFalse(any(e[0]=='read' and e[1]==7 for e in events))

    def test_on_or_hardware_fault_stops_immediately_without_reset_capability(self):
        for address,value in ((64,1),(70,1),(98,255),(144,35),(146,46)):
            bus,r,clock,events=fixture(bad_read=(3,address,value,0,0))
            with self.assertRaises(RuntimeError):self.run_diagnostic(bus,r,clock)
            self.assertFalse(any(e[0]=='ping' and e[1]>3 for e in events))
            self.assertEqual(r['servo_write_commands_sent'],0)

    def test_health_receive_failure_is_recorded_then_other_ids_are_tested(self):
        bus,r,clock,events=fixture(bad_read=(1,70,0,-3002,0))
        self.assertFalse(self.run_diagnostic(bus,r,clock))
        self.assertEqual(sum(e[0]=='ping' for e in events),48)
        self.assertEqual(r['motor_scans'][0]['status'],'health_read_failed')
        self.assertEqual(r['motor_scans'][1]['id'],2)
        self.assertFalse(r['stream_complete'])

    def test_stream_receive_failure_never_returns_old_OFF_as_fresh(self):
        bus,r,clock,events=fixture(stream_comm=-3002)
        with self.assertRaisesRegex(RuntimeError,'Sync READ failed'):self.run_diagnostic(bus,r,clock)
        self.assertFalse(r['all_16_torque_OFF_verified']);self.assertFalse(r['stream_complete'])
        self.assertEqual(sum(e[0]=='sync_read' for e in events),1)

    def test_new_on_or_watchdog_fault_in_stream_is_not_cleared(self):
        for address,value in ((64,1),(98,255),(144,35)):
            bus,r,clock,events=fixture(stream_field=(8,address,value))
            with self.assertRaises(RuntimeError):self.run_diagnostic(bus,r,clock)
            self.assertFalse(r['all_16_torque_OFF_verified'])
            self.assertEqual(sum(e[0]=='sync_read' for e in events),1)

    def test_receive_trace_preserves_result_caps_storage_and_never_retries(self):
        bus,r,clock,events=fixture();bus.install_receive_trace()
        for _ in range(70):self.assertEqual(bus.packet.rxPacket(bus.port,False),([255,255],-3002))
        self.assertEqual(len(r['failed_receive_packets']),64)
        self.assertEqual(r['failed_receive_packets'][0]['received_bytes'],2)
        self.assertEqual(r['failed_receive_packets'][0]['raw_prefix_hex'],'ffff')
        self.assertIsNone(r['failed_receive_packets'][0]['declared_packet_bytes'])
        self.assertEqual(events,[])

    def test_cancel_before_first_ping_has_no_bus_traffic(self):
        bus,r,clock,events=fixture()
        def cancel():raise KeyboardInterrupt('Q')
        with self.assertRaises(KeyboardInterrupt):self.run_diagnostic(bus,r,clock,cancel)
        self.assertEqual(events,[])

    def test_no_execute_creates_no_bus_and_opens_no_devices(self):
        with patch.object(d,'DiagnosticBus',side_effect=AssertionError('Device created')):
            self.assertEqual(d.main([]),0)

    def test_slow_initial_reads_are_not_a_passing_diagnostic(self):
        bus,r,clock,events=fixture();original=bus.packet.ping
        def slow(port,ident):clock.sleep(.201);return original(port,ident)
        bus.packet.ping=slow
        self.assertFalse(self.run_diagnostic(bus,r,clock))
        self.assertFalse(r['scan_timing_within_200ms']);self.assertFalse(r['all_16_torque_OFF_verified'])


if __name__=='__main__':unittest.main()
