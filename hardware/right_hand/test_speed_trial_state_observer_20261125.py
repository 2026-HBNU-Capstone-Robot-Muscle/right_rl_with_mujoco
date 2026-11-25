"""Pure payload observation and fake SDK forwarding; no devices."""
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import speed_trial_state_observer_20261125 as observer


def payload(tick, *, torque=1, watchdog=25, cap=450):
    data = bytearray(83)
    for at, length, value in ((64,1,torque), (98,1,watchdog), (102,2,cap),
                             (120,2,tick), (144,2,50), (146,1,25)):
        data[at-64:at-64+length] = value.to_bytes(length,'little')
    return list(data)


class Tests(unittest.TestCase):
    def test_normal_counter_rollover_is_not_a_reset(self):
        report = {}
        view = observer.MotorStateObserver(report)
        view.observe(7, payload(32750), 10., active=True)
        view.observe(7, payload(57), 10.075, active=True)
        self.assertEqual(view.summary['clock_discontinuities'], [])
        self.assertEqual(view.summary['transitions'], [])

    def test_OFF_RAM_and_clock_change_preserves_both_states_without_claiming_reset(self):
        view = observer.MotorStateObserver({})
        view.observe(7, payload(12000), 10., active=True)
        view.observe(7, payload(100,torque=0,watchdog=0,cap=2352), 10.4, active=True)
        event = view.summary['transitions'][0]
        self.assertTrue(event['clock_discontinuity'])
        self.assertEqual(event['before']['torque'], 1)
        self.assertEqual(event['after']['current_cap'], 2352)
        self.assertTrue(view.summary['clock_discontinuities'][0]['reset_not_confirmed'])

    def test_expected_initialization_and_shutdown_do_not_create_active_faults(self):
        view = observer.MotorStateObserver({})
        view.observe(7, payload(100,torque=0,watchdog=0,cap=2352), 10., active=False)
        view.observe(7, payload(175), 10.075, active=False)
        view.observe(7, payload(250,torque=0,watchdog=0,cap=2352), 10.15, active=False)
        self.assertEqual(view.summary['transitions'], [])

    def test_sdk_forwarder_adds_no_calls_writes_or_result_changes(self):
        data = payload(1234)
        result = (data, 0, 0)
        calls = []

        def init(bus, report, cancel, **kwargs):
            bus.port = object()
            bus.controlled = True
            bus._shutdown_started = False
            bus.packet = SimpleNamespace(readRx=lambda *args:(calls.append(args), result)[1])

        report = {'accepted_start_goal':[2048]*16}
        with patch.object(observer.BasePauseBus, '__init__', init):
            bus = observer.ObservedPauseBus(report, lambda:None)
        self.assertIs(bus.packet.readRx(bus.port,7,83), result)
        self.assertEqual(len(calls),1)
        self.assertEqual(report['motor_state_observer']['observed_packets'],1)
        self.assertEqual(report['motor_state_observer']['additional_motor_reads'],0)
        self.assertEqual(report['motor_state_observer']['additional_motor_writes'],0)


if __name__=='__main__':
    unittest.main()
