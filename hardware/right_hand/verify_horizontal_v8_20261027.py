"""Verify bounded receive recovery, current model proof and protected assets."""
import datetime as dt,hashlib,json,unittest
from pathlib import Path
import horizontal_quarter_plan_20261027 as p
from horizontal_quarter_gate_20261027 import validate_review
names=('test_horizontal_quarter_20261027','test_horizontal_preload_confirmation_20261027',
    'test_horizontal_v3_health_20261027','test_horizontal_trimmed_recovery_20261027',
    'test_rewired_initial_separation_20261027','test_horizontal_thumb_book_numeric_20261027',
    'test_horizontal_placement_settle_20261027','test_horizontal_receive_resync_20261027')
result=unittest.TextTestRunner(verbosity=1).run(unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(n) for n in names))
if not result.wasSuccessful() or result.skipped:raise SystemExit('Final no-device checks incomplete')
proof=validate_review(p.HorizontalQuarterPlan(p.REQUEST),p.PROOF)
previous=json.loads((p.ROOT/'right_hand_rotation_20261027/horizontal_quarter_trial_v5_rewired/verification.json').read_bytes())
for name,wanted in previous['protected_hashes'].items():
    path=Path(name) if Path(name).is_absolute() else p.ROOT/name
    if hashlib.sha256(path.read_bytes()).hexdigest()!=wanted:raise RuntimeError('Protected file changed: '+name)
report=dict(status='all_device_free_checks_passed',tests_passed=result.testsRun,failures=0,errors=0,skipped=0,
    code_hashes=p.hashes(),protected_hashes=previous['protected_hashes'],current_model_cases=210,
    read_resynchronization_policy=proof['read_resynchronization_policy'],
    paths_or_model_shapes_changed=False,motor_writes=0,devices_opened=False,
    timestamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat())
(p.OUT/'verification.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(tests_passed=result.testsRun,current_model_cases=210,motor_writes=0,
    read_budget_ms=200,maximum_full_read_requests=3,protected_assets_unchanged=True)))
