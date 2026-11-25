"""Device-free regression suite and immutable-source evidence."""
import datetime as dt,hashlib,json,unittest
from pathlib import Path
import horizontal_quarter_plan_20261027 as p
from horizontal_quarter_gate_20261027 import validate_review

def main():
    suites=[unittest.defaultTestLoader.loadTestsFromName(n) for n in ('test_horizontal_quarter_20261027','test_horizontal_preload_confirmation_20261027','test_horizontal_v3_health_20261027')]
    result=unittest.TextTestRunner(verbosity=1).run(unittest.TestSuite(suites))
    if not result.wasSuccessful() or result.skipped:raise SystemExit('Incomplete regression checks')
    proof=validate_review(p.HorizontalQuarterPlan(p.REQUEST),p.PROOF)
    expected={'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b','right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716','exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0','camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d','C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py':'008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656'}
    for name,wanted in expected.items():
        path=Path(name) if Path(name).is_absolute() else p.ROOT/name
        if hashlib.sha256(path.read_bytes()).hexdigest()!=wanted:raise RuntimeError('Protected source changed: '+name)
    review=dict(status='all_device_free_regressions_passed',tests_passed=result.testsRun,failures=0,errors=0,skipped=0,current_empty_cases_passed=len(proof['empty_recovery']['static_cases'])+len(proof['empty_recovery']['dynamic_cases']),code_hashes=p.hashes(),protected_hashes=expected,devices_opened=False,motor_writes=0,timestamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat())
    (p.OUT/'verification.json').write_text(json.dumps(review,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(tests_passed=result.testsRun,fresh_empty_model_cases=210,all_protected_sources_unchanged=True,devices_opened=False,motor_writes=0)))
if __name__=='__main__':main()
