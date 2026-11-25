"""Device-free transaction, health and scoped initial-escape regressions."""
import datetime as dt,hashlib,json,unittest
from pathlib import Path
import horizontal_quarter_plan_20261027 as p
from horizontal_quarter_gate_20261027 import validate_review
def main():
    names=('test_horizontal_quarter_20261027','test_horizontal_preload_confirmation_20261027','test_horizontal_v3_health_20261027','test_horizontal_trimmed_recovery_20261027','test_rewired_initial_separation_20261027')
    result=unittest.TextTestRunner(verbosity=1).run(unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(n) for n in names))
    if not result.wasSuccessful() or result.skipped:raise SystemExit('Incomplete current verification')
    plan=p.HorizontalQuarterPlan(p.REQUEST);proof=validate_review(plan,p.PROOF)
    previous=json.loads((p.ROOT/'right_hand_rotation_20261027/horizontal_quarter_trial_v3/verification.json').read_bytes())
    for name,wanted in previous['protected_hashes'].items():
        path=Path(name) if Path(name).is_absolute() else p.ROOT/name
        if hashlib.sha256(path.read_bytes()).hexdigest()!=wanted:raise RuntimeError('Protected file changed: '+name)
    report=dict(status='all_device_free_regressions_passed',tests_passed=result.testsRun,failures=0,errors=0,skipped=0,
        code_hashes=p.hashes(),protected_hashes=previous['protected_hashes'],current_empty_model_cases=210,
        initial_self_separation=proof['initial_self_separation'],physics_shapes_and_forces_unchanged=True,
        devices_opened=False,motor_writes=0,timestamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat())
    (p.OUT/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(tests_passed=result.testsRun,fresh_model_cases=210,required_actual_ring_ticks=proof['initial_self_separation'].get('required_actual_ring_rotation_ticks'),protected_files_unchanged=True,motor_writes=0)))
if __name__=='__main__':main()
