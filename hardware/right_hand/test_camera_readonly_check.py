"""Verify read-only capabilities, timing, cancellation and OFF requirements."""
import unittest
from types import SimpleNamespace
from leap_camera_readonly_check import ReadonlyBus,monitor,validate_off


def snapshot():
    return dict(torque=[0]*16,hardware_error=[0]*16,watchdog=[0]*16,
                position_ticks=[2000]*16,voltage_v=[4.8]*16,temperature_c=[22]*16)


class Clock:
    def __init__(self):self.t=0.
    def now(self):return self.t
    def sleep(self,seconds):self.t+=seconds


class Tests(unittest.TestCase):
    def test_fifty_ms_period_includes_read_and_camera_work(self):
        clock=Clock();report={};reads=[]
        def read():reads.append(1);clock.sleep(.031);return snapshot()
        monitor(read,lambda:clock.sleep(.001),report,cancel=lambda:None,seconds=1.,now=clock.now,sleep=clock.sleep)
        self.assertEqual(report['status'],'readonly_camera_timing_complete')
        self.assertTrue(report['all_observed_torque_off']);self.assertEqual(len(reads),20)
        self.assertAlmostEqual(report['max_read_gap_ms'],50.)
        self.assertAlmostEqual(report['max_camera_poll_ms'],1.)

    def test_slow_gui_or_read_fails_without_any_motor_command_capability(self):
        for delay in ('gui','read'):
            clock=Clock();report={};reads=[]
            def read():reads.append(1);clock.sleep(.25 if delay=='read' else .031);return snapshot()
            with self.assertRaisesRegex(RuntimeError,'200ms'):
                monitor(read,lambda:clock.sleep(.718 if delay=='gui' else .001),report,
                    cancel=lambda:None,seconds=1.,now=clock.now,sleep=clock.sleep)
            self.assertEqual(len(reads),1);self.assertEqual(len(report['samples']),1)

    def test_on_error_and_watchdog_fault_are_not_cleared_or_retried(self):
        for field,value in (('torque',1),('hardware_error',1),('watchdog',255)):
            clock=Clock();report={};reads=[];state=snapshot();state[field][7]=value
            def read():reads.append(1);return state
            with self.assertRaises(RuntimeError):
                monitor(read,lambda:None,report,cancel=lambda:None,seconds=1.,now=clock.now,sleep=clock.sleep)
            self.assertEqual(reads,[1]);self.assertEqual(state[field][7],value)

    def test_cancel_before_read_never_reads_or_sends_commands(self):
        report={};calls=[]
        def cancel():raise KeyboardInterrupt('Q')
        with self.assertRaises(KeyboardInterrupt):monitor(lambda:calls.append('read'),lambda:calls.append('camera'),report,cancel=cancel)
        self.assertEqual(calls,[])

    def test_bus_uses_only_ping_and_sync_read_and_host_port_settings(self):
        events=[]
        class Packet:
            def ping(self,port,ident):events.append(('ping',ident));return 1240,0,0
            def __getattr__(self,name):raise AssertionError('Forbidden packet capability: '+name)
        class Group:
            def addParam(self,ident):events.append(('add_read_id',ident));return True
            def txRxPacket(self):events.append(('sync_read',));return 0
            def isAvailable(self,*args):return True
            def getData(self,ident,address,size):
                return {64:0,70:0,98:0,132:(2**32-1 if ident==1 else 2048),144:48,146:22}[address]
        port=SimpleNamespace(ser=None,openPort=lambda:True,setBaudRate=lambda value:events.append(('host_baud',value)) or True,
                             closePort=lambda:events.append(('close_host_port',)))
        def group(p,k,address,length):self.assertEqual((address,length),(64,83));return Group()
        sdk=SimpleNamespace(COMM_SUCCESS=0,PortHandler=lambda device:port,PacketHandler=lambda proto:Packet(),GroupSyncRead=group)
        bus=ReadonlyBus(sdk,'FAKE');bus.open();state=bus.snapshot();validate_off(state);bus.close()
        self.assertEqual(state['position_ticks'][0],-1)
        self.assertEqual(sum(e[0]=='ping' for e in events),16)
        self.assertEqual(sum(e[0]=='sync_read' for e in events),1)
        self.assertEqual(state['voltage_v'],[4.8]*16)
        self.assertTrue(all(e[0] in ('add_read_id','ping','sync_read','host_baud','close_host_port') for e in events))

    def test_incomplete_read_never_returns_stale_snapshot(self):
        sdk=SimpleNamespace(COMM_SUCCESS=0,PortHandler=lambda d:None,PacketHandler=lambda p:None)
        group=SimpleNamespace(addParam=lambda i:True,txRxPacket=lambda:0,isAvailable=lambda i,a,n:i!=16,
                              getData=lambda *a:(_ for _ in ()).throw(AssertionError('No partial data allowed')))
        sdk.GroupSyncRead=lambda *a:group
        with self.assertRaisesRegex(RuntimeError,'ID16'):
            ReadonlyBus(sdk,'FAKE').snapshot()


if __name__=='__main__':unittest.main()
