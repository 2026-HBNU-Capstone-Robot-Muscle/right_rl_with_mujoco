"""Fault checks for the narrow driver setting and guarded restart path."""
import contextlib,io,json,tempfile,unittest
from pathlib import Path
from unittest.mock import MagicMock,patch
import set_com7_usb_latency_20261027 as setter
import usb_latency_admin_workflow_20261027 as workflow

ROOT=Path(__file__).resolve().parent
ENTRY=dict(port='COM7',latency_timer_ms=16,
    registry_path='SYSTEM\\CurrentControlSet\\Enum\\FTDIBUS\\VID_0403+PID_6014+TEST\\0000\\Device Parameters')

class DriverSettingTests(unittest.TestCase):
    def perform(self,*,torque=0,baud=5,readback=1):
        bus=MagicMock();bus.snapshot.return_value=dict(torque=[torque]*16,hardware_error=[0]*16,watchdog=[0]*16,
            position_ticks=[2000]*16,voltage_v=[5.]*16,temperature_c=[20]*16)
        bus.packet.read1ByteTxRx.return_value=(baud,0,0)
        with tempfile.TemporaryDirectory(prefix='usb_latency_change_test_',dir=ROOT) as folder:
            with patch('sys.argv',['setter','--apply','--out',folder]),patch('leap_camera_readonly_check.ReadonlyBus',return_value=bus),\
                patch('benchmark_current_serial_readonly_20261027.inspect_usb_latency',return_value=dict(matches=[dict(ENTRY)])),\
                patch('winreg.OpenKey') as opening,patch('winreg.QueryValueEx',side_effect=[(16,4),(readback,4)]),\
                patch('winreg.SetValueEx') as writing,contextlib.redirect_stdout(io.StringIO()):
                result=setter.main();report=json.loads((Path(folder)/'result.json').read_text())
                writes=[call.args[-1] for call in writing.call_args_list]
                return result,report,writes,opening.call_count

    def test_torque_on_prevents_every_driver_write(self):
        result,report,writes,opens=self.perform(torque=1)
        self.assertEqual(result,1);self.assertEqual(writes,[]);self.assertEqual(opens,0)
        self.assertEqual(report['motor_writes'],0)

    def test_unexpected_motor_baud_prevents_driver_write(self):
        result,report,writes,opens=self.perform(baud=6)
        self.assertEqual(result,1);self.assertEqual(writes,[]);self.assertEqual(opens,0)

    def test_healthy_off_changes_only_latency_once(self):
        result,report,writes,opens=self.perform()
        self.assertEqual(result,0);self.assertEqual(writes,[1]);self.assertEqual(report['baud_writes'],0)
        self.assertFalse(report['measured_read_speed_improvement_verified'])

    def test_failed_readback_restores_original(self):
        result,report,writes,opens=self.perform(readback=2)
        self.assertEqual(result,1);self.assertEqual(writes,[1,16]);self.assertEqual(report['rolled_back_to_ms'],16)

class WorkflowTests(unittest.TestCase):
    def test_restart_instance_cannot_match_multiple_devices(self):
        self.assertTrue(workflow.device_instance(ENTRY).startswith('FTDIBUS\\'))
        for bad in (dict(ENTRY,port='COM8'),dict(ENTRY,registry_path=ENTRY['registry_path'].replace('TEST','*')),
                    dict(ENTRY,registry_path=ENTRY['registry_path'].replace('FTDIBUS','USB'))):
            with self.assertRaises(RuntimeError):workflow.device_instance(bad)

    def test_benefit_requires_all_healthy_reads_and_same_baud(self):
        data=dict(status='100_full_off_reads_complete',communication_failures=0,all16_baud_value5_3mbps=True,
            frames=[{}]*100,maximum_read_ms=15,minimum_voltage_v=5.,median_read_ms=10)
        baseline=dict(median_read_ms=26.)
        self.assertTrue(workflow.accepted(data,baseline))
        for change in (dict(median_read_ms=25),dict(communication_failures=1),dict(minimum_voltage_v=3.5),
                       dict(maximum_read_ms=100),dict(all16_baud_value5_3mbps=False),dict(frames=[{}]*99)):
            self.assertFalse(workflow.accepted(dict(data,**change),baseline))

    def test_default_requests_no_admin_and_opens_no_devices(self):
        with patch('sys.argv',['workflow']),patch.object(workflow,'elevate') as admin,\
             patch.object(workflow,'run_admin') as change,contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(workflow.main(),0);admin.assert_not_called();change.assert_not_called()

    def test_unhealthy_post_read_does_not_restart_and_restores_registry(self):
        with tempfile.TemporaryDirectory(prefix='usb_latency_change_test_',dir=ROOT) as folder:
            out=Path(folder)
            def applied(*args,**kwargs):
                (out/'result.json').write_text(json.dumps(dict(status='driver_registry_updated',registry_writes=1,original=ENTRY)))
                return MagicMock(returncode=0,stdout=b'',stderr=b'')
            with patch('ctypes.windll.shell32.IsUserAnAdmin',return_value=1),\
                 patch.object(workflow.subprocess,'run',side_effect=applied),\
                 patch.object(workflow,'benchmark',return_value=dict(status='stopped',error='Torque ON')),\
                 patch.object(workflow,'restart_one') as restart,patch('winreg.OpenKey'),patch('winreg.SetValueEx') as restore,\
                 patch.object(workflow,'LATEST',out/'latest.json'):
                self.assertEqual(workflow.run_admin(out),1);restart.assert_not_called()
                self.assertEqual(restore.call_args.args[-1],16)

if __name__=='__main__':unittest.main()
