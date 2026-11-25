"""Offline checks for the torque-OFF-only watchdog restoration."""
import unittest
from unittest.mock import Mock

import leap_restore_watchdog


class FakePacket:
    def __init__(self,watchdog=None,torque=None):
        self.watchdog={i:0 for i in range(1,17)}
        self.watchdog.update(watchdog or {})
        self.torque={i:0 for i in range(1,17)}
        self.torque.update(torque or {})
        self.writes=[]

    def ping(self,port,motor):
        return 1240,0,0

    def read1ByteTxRx(self,port,motor,address):
        values={64:self.torque[motor],70:0,98:self.watchdog[motor],146:25}
        return values[address],0,0

    def read2ByteTxRx(self,port,motor,address):
        return 50,0,0

    def write1ByteTxRx(self,port,motor,address,value):
        self.writes.append((motor,address,value))
        if address!=98 or value!=0:
            raise AssertionError('Unexpected register write')
        self.watchdog[motor]=0
        return 0,0


class Tests(unittest.TestCase):
    def make_sdk(self,packet):
        port=Mock()
        port.openPort.return_value=True
        port.setBaudRate.return_value=True
        sdk=Mock(COMM_SUCCESS=0, PortHandler=Mock(return_value=port),
                 PacketHandler=Mock(return_value=packet))
        return sdk,port

    def test_restores_only_stale_idle_watchdogs(self):
        packet=FakePacket(watchdog={7:25,9:25,16:25})
        sdk,port=self.make_sdk(packet)
        report={}
        self.assertEqual(leap_restore_watchdog.run(sdk,report),0)
        self.assertEqual(packet.writes,[(7,98,0),(9,98,0),(16,98,0)])
        self.assertEqual(len(report['postflight']),16)
        self.assertTrue(report['all_torque_off_verified'])
        self.assertTrue(all(x['bus_watchdog_raw']==0 for x in report['postflight']))
        port.closePort.assert_called_once()

    def test_any_torque_on_blocks_all_writes(self):
        packet=FakePacket(watchdog={7:25},torque={16:1})
        sdk,port=self.make_sdk(packet)
        with self.assertRaisesRegex(RuntimeError,'torque ON'):
            leap_restore_watchdog.run(sdk,{})
        self.assertEqual(packet.writes,[])
        port.closePort.assert_called_once()

    def test_watchdog_error_value_is_not_cleared(self):
        packet=FakePacket(watchdog={7:255})
        sdk,port=self.make_sdk(packet)
        with self.assertRaisesRegex(RuntimeError,'expected 0 or 25'):
            leap_restore_watchdog.run(sdk,{})
        self.assertEqual(packet.writes,[])
        port.closePort.assert_called_once()
