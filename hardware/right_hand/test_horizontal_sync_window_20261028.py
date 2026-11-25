"""Inject delayed responses without changing health or motion behavior."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
from horizontal_sync_receive_window_20261028 import block_once

class Group:
    def __init__(self,port,delay,tx_result=0):
        self.port=port;self.delay=delay;self.tx_result=tx_result;self.data_dict={};self.cleared=False;self.rx_called=False
    def addParam(self,i):self.data_dict[i]=[];return True
    def txPacket(self):self.port.window_ms=39.;return self.tx_result
    def rxPacket(self):
        self.rx_called=True;good=self.delay<=self.port.window_ms
        for i in self.data_dict:self.data_dict[i]=[0]*self.port.length if good or i<9 else []
        return 0 if good else -3001
    def isAvailable(self,i,a,n):return len(self.data_dict[i])==self.port.length
    def getData(self,i,a,n):return i
    def clearParam(self):self.cleared=True;self.data_dict.clear()

class WindowTests(unittest.TestCase):
    def make(self,delay,length=83,tx_result=0):
        port=SimpleNamespace(window_ms=0.,length=length)
        port.setPacketTimeoutMillis=lambda v:setattr(port,'window_ms',v)
        group=Group(port,delay,tx_result)
        bus=SimpleNamespace(sdk=SimpleNamespace(COMM_SUCCESS=0,GroupSyncRead=lambda *a:group),
            port=port,packet=object(),report={},write=Mock())
        return bus,group
    def test_normal_complete_read_returns_all16(self):
        bus,g=self.make(15);values,comm,error,detail=block_once(bus,64,83,[('torque',64,1)])
        self.assertEqual(values['torque'].tolist(),list(range(1,17)));self.assertEqual(comm,0);self.assertTrue(g.cleared)
        bus.write.assert_not_called()
    def test_complete55ms_response_is_not_cut_off_at39ms(self):
        bus,g=self.make(55);values,comm,_,_=block_once(bus,64,83,[('voltage',144,2)])
        self.assertEqual(comm,0);self.assertEqual(len(values['voltage']),16);self.assertEqual(bus.port.window_ms,80)
    def test_missing_response_after80ms_is_rejected(self):
        bus,g=self.make(90);values,comm,_,detail=block_once(bus,64,83,[('position',132,4)])
        self.assertIsNone(values);self.assertEqual(comm,-3001)
        self.assertIn(detail['first_incomplete_id'],range(1,17));self.assertTrue(g.cleared)
    def test_torque_off_readback_has_the_same_bounded_window(self):
        bus,g=self.make(55,length=1);values,comm,_,_=block_once(bus,64,1,[('torque',64,1)])
        self.assertEqual(comm,0);self.assertEqual(len(values['torque']),16)
    def test_other_register_does_not_receive_a_longer_window(self):
        bus,g=self.make(10)
        with self.assertRaises(ValueError):block_once(bus,98,1,[('watchdog',98,1)])
        self.assertFalse(g.rx_called)
    def test_failed_read_transmission_never_calls_receive(self):
        bus,g=self.make(15,tx_result=-1001);_,comm,_,_=block_once(bus,64,83,[('torque',64,1)])
        self.assertEqual(comm,-1001);self.assertFalse(g.rx_called);self.assertTrue(g.cleared)
    def test_current_bus_routes_only_health_and_off_readback_to_the_new_window(self):
        import leap_horizontal_quarter_20261027 as live
        bus=live.HorizontalQuarterBus.__new__(live.HorizontalQuarterBus)
        with patch.object(live,'windowed_block',return_value='new') as new,patch.object(live.NewPairBus,'_block_once',return_value='old') as old:
            self.assertEqual(bus._block_once(64,83,[]),'new')
            self.assertEqual(bus._block_once(64,1,[]),'new')
            self.assertEqual(bus._block_once(98,1,[]),'old')
            self.assertEqual(new.call_count,2);self.assertEqual(old.call_count,1)
if __name__=='__main__':unittest.main()
