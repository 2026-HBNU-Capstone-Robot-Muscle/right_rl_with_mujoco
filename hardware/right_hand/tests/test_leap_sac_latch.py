"""The SAC launcher must not move after an unverified torque-OFF event."""
import json
import os
import tempfile
import unittest
from pathlib import Path

import leap_sac


class Tests(unittest.TestCase):
    @staticmethod
    def write(path, data, stamp):
        path.write_text(json.dumps(data), encoding='utf-8')
        os.utime(path, (stamp, stamp))

    @staticmethod
    def diagnostic_rows():
        return [dict(id=i, model_number=1240, torque_enabled=0, torque_enabled_end=0,
                     hardware_error=0, bus_watchdog_raw=0) for i in range(1,17)]

    def test_unverified_off_requires_new_complete_inspection(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            failed=root/'leap_sac_20261005_200411.json'
            self.write(failed, {'all_torque_off_verified': False}, 100)
            # A later partial OFF result is not sufficient.
            self.write(root/'leap_sac_result.json',
                       {'all_torque_off_verified': True, 'final_torque_states': [0]*9}, 101)
            self.assertEqual(leap_sac.uncleared_off_failure(root), failed)
            diag=root/'hand_readonly_check_result.json'
            good=dict(status='read_complete_not_motion_approval', motors=self.diagnostic_rows())
            self.write(diag, good, 99)
            self.assertEqual(leap_sac.uncleared_off_failure(root), failed)
            bad=dict(good, motors=self.diagnostic_rows())
            bad['motors'][6]['bus_watchdog_raw']=255
            self.write(diag, bad, 102)
            self.assertEqual(leap_sac.uncleared_off_failure(root), failed)
            self.write(diag, good, 103)
            self.assertIsNone(leap_sac.uncleared_off_failure(root))

    def test_new_unverified_off_rearms_latch(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            self.assertIsNone(leap_sac.uncleared_off_failure(root))
            self.write(root/'hand_readonly_check_result.json',
                       dict(status='read_complete_not_motion_approval',motors=self.diagnostic_rows()), 100)
            failed=root/'leap_sac_20261005_200411.json'
            self.write(failed, {'all_torque_off_verified': False}, 101)
            self.assertEqual(leap_sac.uncleared_off_failure(root), failed)

    def test_verified_idle_restore_also_clears_latch(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            failed=root/'leap_sac_20261005_200411.json'
            self.write(failed, {'all_torque_off_verified': False}, 100)
            restore=root/'leap_restore_watchdog_result.json'
            verified=dict(status='idle_watchdog_restored_verified',
                          all_torque_off_verified=True,postflight=self.diagnostic_rows())
            self.write(restore,verified,101)
            self.assertIsNone(leap_sac.uncleared_off_failure(root))
            verified['postflight'][6]['bus_watchdog_raw']=25
            self.write(restore,verified,102)
            self.assertEqual(leap_sac.uncleared_off_failure(root), failed)
