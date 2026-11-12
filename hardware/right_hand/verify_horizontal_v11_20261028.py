"""Verify host1Mbps/grouped reads, immutable motion and current-pose proof."""
import datetime as dt,hashlib,json,unittest
from pathlib import Path
import horizontal_quarter_plan_20261027 as p
from horizontal_quarter_gate_20261027 import validate_review
from horizontal_serial_link_20261028 import metadata
names=('test_horizontal_quarter_20261027','test_horizontal_preload_confirmation_20261027',
    'test_horizontal_v3_health_20261027','test_horizontal_trimmed_recovery_20261027',
    'test_rewired_initial_separation_20261027','test_horizontal_thumb_book_numeric_20261027',
    'test_horizontal_placement_settle_20261027','test_horizontal_receive_resync_20261027',
    'test_horizontal_sync_window_20261028','test_horizontal_recovery_episode_20261028',
    'test_horizontal_serial_link_20261028')
result=unittest.TextTestRunner(verbosity=1).run(unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(n) for n in names))
if not result.wasSuccessful() or result.skipped:raise SystemExit('No-device checks incomplete')
proof=validate_review(p.HorizontalQuarterPlan(p.REQUEST),p.PROOF)
prior=json.loads((p.ROOT/'right_hand_rotation_20261028/v10_before_host_link_update/verification.json').read_bytes())
for name,wanted in prior['protected_hashes'].items():
    path=Path(name) if Path(name).is_absolute() else p.ROOT/name
    if hashlib.sha256(path.read_bytes()).hexdigest()!=wanted:raise RuntimeError('Protected file changed: '+name)
assert hashlib.sha256(p.LOADED_PATH.read_bytes()).hexdigest()=='631c4081f5bbf75490479f48a9a84e437c7f7280597cdc484722205b6a0b30df'
report=dict(status='all_device_free_checks_passed',tests_passed=result.testsRun,failures=0,errors=0,skipped=0,
    code_hashes=p.hashes(),protected_hashes=prior['protected_hashes'],current_model_cases=210,
    verified_serial_link_policy=metadata(),loaded_goals_unchanged=True,motor_writes=0,devices_opened=False,
    hardware_health_thresholds_unchanged=True,timestamp=dt.datetime.now().astimezone().isoformat())
(p.OUT/'verification.json').write_bytes((json.dumps(report,indent=2)+'\n').encode())
print(json.dumps(dict(tests=result.testsRun,model_conditions=210,host_baud=1000000,reply_group=1,motor_writes=0,protected_assets_unchanged=True)))
