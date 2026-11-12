"""Bounded inspection of operator trial records; opens no devices."""
import json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parent
for folder in sorted(ROOT.glob('supported_inverse_empty_20261029_*')):
    path=folder/'result.json'
    if not path.exists():print(json.dumps(dict(folder=folder.name,result_exists=False)));continue
    r=json.loads(path.read_bytes());samples=r.get('samples',[])
    print(json.dumps(dict(folder=folder.name,source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        keys=list(r),status=r.get('status'),error=r.get('error'),stop=r.get('stop_reason'),
        all_torque_off_verified=r.get('all_torque_off_verified'),settings_restored=r.get('settings_restored'),
        hardware_authorized=r.get('hardware_authorized'),samples=len(samples),write_events=len(r.get('write_events',[])),
        phase_counts={p:sum(s['phase']==p for s in samples) for p in dict.fromkeys(s['phase'] for s in samples)},
        last_sample=samples[-1] if samples else None,camera_recording=r.get('camera_recording'),
        read_failures=r.get('read_failures'),accepted_enable_goal=r.get('accepted_enable_goal'),
        final_off_reference_valid=r.get('final_off_reference_valid')),ensure_ascii=False))
