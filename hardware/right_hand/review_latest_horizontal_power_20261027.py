"""Bounded current operator-run summary; no stale pasted text or devices."""
import json
from collections import Counter
from pathlib import Path
ROOT=Path(__file__).resolve().parent
for folder in sorted(ROOT.glob('horizontal_quarter_trial_20261027_*'))[-2:]:
    r=json.loads((folder/'result.json').read_bytes());samples=r.get('samples',[])
    out=dict(folder=folder.name,status=r.get('status'),error=r.get('error'),failed_phase=r.get('failed_phase'),phases=dict(Counter(s['phase'] for s in samples)),off_verified=r.get('all_torque_off_verified'),settings_restored=r.get('settings_restored'),voltage_events=r.get('voltage_events',[]))
    if samples:
        out.update(minimum_voltage_v=min(v for s in samples for v in s['voltage_v']),last_sample=samples[-1])
    if r.get('current_start_screen'):
        proof=json.loads(Path(r['current_start_screen']).read_bytes());out['plan_version']=proof.get('plan_version')
    print(json.dumps(out,ensure_ascii=False))
