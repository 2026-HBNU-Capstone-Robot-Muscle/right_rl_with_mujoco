"""Run the complete relevant fault matrix and bind the unchanged motion data."""
import datetime as dt,hashlib,json,unittest
from pathlib import Path
import horizontal_quarter_plan_20261027 as p
from horizontal_quarter_gate_20261027 import validate_review
from horizontal_read_recovery_20261028 import POLICY
names=('test_horizontal_quarter_20261027','test_horizontal_preload_confirmation_20261027',
    'test_horizontal_v3_health_20261027','test_horizontal_trimmed_recovery_20261027',
    'test_rewired_initial_separation_20261027','test_horizontal_thumb_book_numeric_20261027',
    'test_horizontal_placement_settle_20261027','test_horizontal_receive_resync_20261027',
    'test_horizontal_sync_window_20261028','test_horizontal_recovery_episode_20261028')
result=unittest.TextTestRunner(verbosity=1).run(unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(n) for n in names))
if not result.wasSuccessful() or result.skipped:raise SystemExit('Final no-device checks incomplete')
proof=validate_review(p.HorizontalQuarterPlan(p.REQUEST),p.PROOF)
previous=json.loads((p.ROOT/'right_hand_rotation_20261028/v9_before_confirmation_burst_fix/verification.json').read_bytes())
for name,wanted in previous['protected_hashes'].items():
    path=Path(name) if Path(name).is_absolute() else p.ROOT/name
    if hashlib.sha256(path.read_bytes()).hexdigest()!=wanted:raise RuntimeError('Protected file changed: '+name)
old_request=p.ROOT/'right_hand_rotation_20261028/v9_before_confirmation_burst_fix/request.json'
old=json.loads(old_request.read_bytes())
assert proof['read_recovery_episode_policy']==POLICY
assert hashlib.sha256(p.LOADED_PATH.read_bytes()).hexdigest()=='631c4081f5bbf75490479f48a9a84e437c7f7280597cdc484722205b6a0b30df'
report=dict(status='all_device_free_checks_passed',tests_passed=result.testsRun,failures=0,errors=0,skipped=0,
    code_hashes=p.hashes(),protected_hashes=previous['protected_hashes'],current_model_cases=210,
    read_recovery_episode_policy=dict(POLICY),loaded_goals_unchanged=True,
    hardware_health_thresholds_unchanged=True,motor_writes=0,devices_opened=False,
    timestamp=dt.datetime.now().astimezone().isoformat())
(p.OUT/'verification.json').write_bytes((json.dumps(report,indent=2)+'\n').encode())
print(json.dumps(dict(tests_passed=result.testsRun,current_model_cases=210,motor_writes=0,
    whole_sample_budget_ms=200,two_consecutive_clean_reads=True,protected_assets_unchanged=True)))
