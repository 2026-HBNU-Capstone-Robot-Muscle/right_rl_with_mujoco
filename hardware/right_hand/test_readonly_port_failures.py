"""Offline regressions: no serial ports or cameras opened."""
import unittest
from types import SimpleNamespace
from hand_readonly_check import inspect_bus, require_serial_port


class PortFailureTests(unittest.TestCase):
    def inspect(self, port, report):
        class Packet:
            def ping(self, *args):
                raise RuntimeError('original read failure')
        sdk = SimpleNamespace(PortHandler=lambda _: port,
                              PacketHandler=lambda _: Packet(), COMM_SUCCESS=0)
        inspect_bus(sdk, 'FAKE', report)

    def test_missing_port_inventory_does_not_select_bluetooth(self):
        report = {}
        ports = [SimpleNamespace(device='COM3', description='Bluetooth', vid=None, pid=None)]
        with self.assertRaisesRegex(RuntimeError, 'COM7 not present'):
            require_serial_port('COM7', report, lambda: ports)
        self.assertEqual(report['available_ports'][0]['device'], 'COM3')
        require_serial_port('com3', report, lambda: ports)

    def test_failed_open_never_closes_none(self):
        for raises in (True, False):
            class Port:
                ser = None
                def openPort(self):
                    if raises:
                        raise OSError('original open failure')
                    return False
                def closePort(self):
                    self.fail_if_called = True
                    raise AssertionError('must not close an unopened port')
            port = Port()
            with self.assertRaisesRegex((OSError, RuntimeError),
                                        'original open failure' if raises else 'Cannot open'):
                self.inspect(port, {'motors': []})
            self.assertFalse(hasattr(port, 'fail_if_called'))

    def test_partial_open_handle_is_closed(self):
        closed = []
        class Port:
            ser = None
            def openPort(self):
                self.ser = object()
                raise OSError('setup failed after handle created')
            def closePort(self):
                closed.append(True)
        with self.assertRaisesRegex(OSError, 'setup failed'):
            self.inspect(Port(), {'motors': []})
        self.assertEqual(closed, [True])

    def test_cleanup_failure_does_not_mask_read_failure(self):
        class Port:
            def openPort(self): return True
            def setBaudRate(self, _): return True
            def closePort(self): raise OSError('cleanup failure')
        report = {'motors': []}
        with self.assertRaisesRegex(RuntimeError, 'original read failure'):
            self.inspect(Port(), report)
        self.assertEqual(report['port_close_error'], 'cleanup failure')


if __name__ == '__main__':
    unittest.main()
