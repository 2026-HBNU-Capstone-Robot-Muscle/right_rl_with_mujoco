"""Verify that the user-owned offline launcher cannot start a motion entry."""
import contextlib,io,json,subprocess,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import run_precise_simulation_review_20261028 as launch
class LauncherTests(unittest.TestCase):
    def test_model_command_has_only_offline_checker_and_workspace_evidence(self):
        command=launch.model_command()
        self.assertEqual(command[:4],['wsl','--exec','/home/star1209042/venv/bin/python','-B'])
        self.assertTrue(command[4].endswith('/screen_calibrated_quarter_20261028.py'))
        self.assertNotIn('--execute',command);self.assertNotIn('--read',command)
        self.assertTrue(command[5].endswith('/request.json'));self.assertTrue(command[6].endswith('/review.json'))
    def test_default_checks_sources_without_starting_any_subprocess(self):
        with patch.object(launch.sys,'argv',['check']),patch.object(launch.subprocess,'run',side_effect=AssertionError('subprocess forbidden')),contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(launch.main(),0)
    def test_failed_simulation_stops_before_any_further_program(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp)
            command=['wsl','--exec','offline_checker']
            with patch.object(launch.p,'OUT',folder),patch.object(launch.p,'PROOF',folder/'review.json'),patch.object(launch,'check_sources',return_value={}),patch.object(launch,'model_command',return_value=command),patch.object(launch.sys,'argv',['check','--run-simulation']),patch.object(launch.subprocess,'run',side_effect=subprocess.CalledProcessError(1,command)) as run,contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(launch.main(),2)
            self.assertEqual(run.call_count,1)
            report=json.loads((folder/'local_simulation_check_result.json').read_bytes())
            self.assertEqual(report['status'],'simulation_or_test_review_stopped');self.assertEqual(report['motor_writes'],0)
if __name__=='__main__':unittest.main()
