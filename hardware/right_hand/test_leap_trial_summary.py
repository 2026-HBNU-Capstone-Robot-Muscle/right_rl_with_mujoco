"""Saved-result reporting must not mistake an old success for a new run."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import leap_trial_summary as s


class Tests(unittest.TestCase):
    def test_actual_completed_trial_retains_unverified_physical_grasp(self):
        result=s.read_result(s.ROOT/'cube_grasp_trial_20261022_141534_666300/result.json')
        self.assertTrue(result['approach_sequence_completed'])
        self.assertTrue(result['all_torque_off_verified'] and result['settings_restored'])
        self.assertTrue(result['cube_removal_confirmed'])
        self.assertFalse(result['physical_grasp_verified'] or result['physical_rotation_verified'])

    def test_no_new_result_cannot_show_previous_success(self):
        with tempfile.TemporaryDirectory() as folder:
            result=Path(folder)/'result.json';marker=Path(folder)/'begin.json'
            result.write_text(json.dumps({'status':'cube_grasp_approach_trial_complete'}),encoding='utf-8')
            s.begin(result,marker)
            with self.assertRaisesRegex(RuntimeError,'새 결과가 없습니다'):s.read_result(result,marker)
            result.unlink()
            with self.assertRaises(OSError):s.read_result(result,marker)

    def test_new_stopped_result_does_not_inherit_previous_success(self):
        with tempfile.TemporaryDirectory() as folder:
            result=Path(folder)/'result.json';marker=Path(folder)/'begin.json'
            result.write_text(json.dumps({'status':'cube_grasp_approach_trial_complete'}),encoding='utf-8')
            s.begin(result,marker)
            result.write_text(json.dumps({'status':'stopped','error':'read failed','all_torque_off_verified':True}),encoding='utf-8')
            saved=s.read_result(result,marker)
            self.assertFalse(saved['approach_sequence_completed']);self.assertEqual(saved['error'],'read failed')
            self.assertIsNone(saved['min_voltage_v'])


if __name__=='__main__':unittest.main()
