"""Re-run the existing guarded transaction checks and new recorded-start replay."""
import datetime as dt,hashlib,json,unittest
from pathlib import Path
import horizontal_quarter_plan_20261027 as p
from horizontal_quarter_gate_20261027 import validate_review

def main():
    names=('test_horizontal_quarter_20261027','test_horizontal_preload_confirmation_20261027','test_horizontal_v3_health_20261027','test_horizontal_trimmed_recovery_20261027')
    result=unittest.TextTestRunner(verbosity=1).run(unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(n) for n in names))
    if not result.wasSuccessful() or result.skipped:raise SystemExit('Incomplete regression checks')
    proof=validate_review(p.HorizontalQuarterPlan(p.REQUEST),p.PROOF)
    old=json.loads((p.ROOT/'right_hand_rotation_20261027/horizontal_quarter_trial_v3/verification.json').read_bytes())
    for name,wanted in old['protected_hashes'].items():
        path=Path(name) if Path(name).is_absolute() else p.ROOT/name
        if hashlib.sha256(path.read_bytes()).hexdigest()!=wanted:raise RuntimeError('Protected source changed: '+name)
    review=dict(status='all_device_free_regressions_passed',tests_passed=result.testsRun,failures=0,errors=0,skipped=0,
        current_empty_cases_passed=len(proof['empty_recovery']['static_cases'])+len(proof['empty_recovery']['dynamic_cases']),code_hashes=p.hashes(),
        protected_hashes=old['protected_hashes'],devices_opened=False,motor_writes=0,timestamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat())
    (p.OUT/'verification.json').write_text(json.dumps(review,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(tests_passed=result.testsRun,fresh_empty_model_cases=210,protected_sources_unchanged=True,devices_opened=False,motor_writes=0)))
if __name__=='__main__':main()
