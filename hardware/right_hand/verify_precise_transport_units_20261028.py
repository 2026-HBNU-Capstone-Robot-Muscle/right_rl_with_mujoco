"""Device-free regressions that do not need the pending current-start model."""
import datetime as dt,json,unittest
import calibrated_quarter_plan_20261028 as p
from calibrated_quarter_gate_20261028 import validate_loaded
from calibrated_precise_clock_20261028 import review
names=('test_horizontal_quarter_20261027','test_horizontal_preload_confirmation_20261027',
 'test_horizontal_v3_health_20261027','test_horizontal_trimmed_recovery_20261027',
 'test_rewired_initial_separation_20261027','test_horizontal_thumb_book_numeric_20261027',
 'test_horizontal_placement_settle_20261027','test_horizontal_receive_resync_20261027',
 'test_horizontal_sync_window_20261028','test_horizontal_recovery_episode_20261028',
 'test_horizontal_serial_link_20261028','test_single_reply40_20261028','test_precise_two_full_20261028')
result=unittest.TextTestRunner(verbosity=1).run(unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(n) for n in names))
if not result.wasSuccessful() or result.skipped:raise SystemExit('Device-free regressions failed')
loaded,static=validate_loaded();evidence=review()
report=dict(status='device_free_checks_passed',tests=result.testsRun,failures=0,errors=0,skipped=0,
    loaded_physics_cases=len(loaded['cases']),loaded_static_cases=len(static['cases']),
    precise_readonly_evidence=evidence,current_start_model_review_complete=p.PROOF.exists(),
    devices_opened=False,motor_writes=0,timestamp=dt.datetime.now().astimezone().isoformat())
(p.OUT/'transport_unit_checks.json').write_bytes((json.dumps(report,indent=2)+'\n').encode())
print(json.dumps(report))
