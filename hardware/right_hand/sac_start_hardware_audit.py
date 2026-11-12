"""Summarize archived revised-start hardware trials; never opens devices."""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
rows=[]
for path in sorted(ROOT.glob('leap_sac_start_empty_20*.json')):
    r=json.loads(path.read_text(encoding='utf-8'));samples=r.get('samples',[])
    row={k:r.get(k) for k in ('timestamp','status','error','failed_phase','all_torque_off_verified','settings_restored')}
    row['file']=path.name;row['samples']=len(samples)
    if samples:
        voltage=np.asarray([s['voltage_v'] for s in samples]);currents=np.asarray([s['current_ma'] for s in samples]);errors=np.asarray([s['tracking_rad'] for s in samples])
        row.update(min_voltage_v=float(voltage.min()),peak_current_ma=float(np.abs(currents).max()),max_tracking_rad=float(errors.max()),
                   min_voltage_by_id=voltage.min(0).tolist(),last_sample=samples[-1],low_voltage_samples=[s for s in samples if min(s['voltage_v'])<3.7],
                   worst_voltage_sample=samples[int(np.argmin(voltage.min(1)))],initial_voltage_raw=r.get('initial',{}).get('voltage'))
    rows.append(row)
report=dict(status='saved_hardware_logs_reviewed',motor_commands_sent=0,trials=rows)
(ROOT/'sac_start_hardware_audit_result.json').write_text(json.dumps(report,indent=2)+'\n')
for row in rows:
    print(json.dumps({k:v for k,v in row.items() if k not in ('last_sample','low_voltage_samples','worst_voltage_sample')},ensure_ascii=False))
    if row.get('last_sample'):
        s=row['worst_voltage_sample'];print(json.dumps(dict(worst_voltage_phase=s['phase'],voltage=s['voltage_v'],current_ma=s['current_ma'],read_ms=s['read_ms'],low_voltage_samples=len(row['low_voltage_samples']))))
