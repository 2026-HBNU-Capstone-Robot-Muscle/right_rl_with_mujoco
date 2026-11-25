import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
r=json.loads((ROOT/'reverse_quarter_resume_empty_20261025_090335_740374/result.json').read_bytes())
for key in ('final_off_reference_valid','final_off_endpoint_drift_ticks','on_to_off_position_change_ticks'):
    print(key,r.get(key))
for i,s in enumerate(r.get('final_off_snapshots',[])):
    print('OFF',i,s.get('position'),list(s))
print('PREFLIGHT_KEYS',list(r['preflight_state']))
print('TIMESTEP',r.get('requested_sample_period_s'))
