"""Run offline regressions. Never opens a camera or serial port."""
import contextlib
import io
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent/'work'))
sys.path.insert(0, str(ROOT/'tests'))

if __name__ == '__main__':
    modules = ['test_vision_diagnostics', 'test_loaded_policy_trial', 'test_hand_prepare_full', 'test_hand_prepare_contact', 'test_fixed_cube_contact', 'test_marker_detector', 'test_bounded_policy', 'test_hand_prepare_resume', 'test_camera_stream', 'test_leap_resume', 'test_hand_start_pose', 'test_hand_recover_then_start',
               'test_encoder_policy_shadow', 'test_held_policy_shadow',
               'test_hand12_start_target_80', 'test_hand12_start_target_100',
               'test_hand_pip_inward_recovery', 'test_hand_tip_settings_readonly']
    modules.append('test_preparation_watchdog')
    modules.append('test_loaded_policy_010')
    modules.append('test_policy_vision_evidence')
    modules.append('test_leap_sac')
    modules.append('test_read_timeout_recovery')
    modules.append('test_leap_sac_latch')
    modules.append('test_leap_restore_watchdog')
    suite = unittest.defaultTestLoader.loadTestsFromNames(modules)
    with contextlib.redirect_stdout(io.StringIO()):
        result = unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(0 if result.wasSuccessful() else 1)
