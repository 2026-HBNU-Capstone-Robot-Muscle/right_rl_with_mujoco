import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
name='support_first_trial_20261113_042739_709487'
r=json.loads((ROOT/name/'result.json').read_bytes())
print(json.dumps({k:r.get(k) for k in ['status','error','failed_phase','primary_failed_phase','last_issued_phase','cube_placement_confirmed','all_torque_off_verified','settings_restored']},ensure_ascii=False))
failures=r.get('read_failures',[])
for failure in failures:
    print('READ_FAILURE',json.dumps({k:failure.get(k) for k in ['kind','address','length','first_incomplete_id','requested_ids','completed_ids','receive_window_ms','received_packet_bytes','expected_packet_bytes','attempt','comm','elapsed_ms','recovered','retry_requested','total_elapsed_ms','remaining_budget_after_quiet_ms']},ensure_ascii=False))
samples=r.get('samples',[])
if samples:
    s=samples[-1]
    print('LAST_HEALTH',json.dumps(dict(phase=s.get('phase'),time=s.get('t'),max_current_ma=max(abs(x) for x in s.get('current_ma',[0])),max_tracking_rad=max(s.get('tracking_rad',[0])),voltage=s.get('voltage_v'),motor_errors=s.get('error'),read_ms=s.get('read_ms')),ensure_ascii=False))
print('POLICY',json.dumps(r.get('read_resynchronization_policy'),ensure_ascii=False))
print('FAILURE_COUNT',len(failures),'SAMPLES',len(samples))
