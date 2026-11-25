"""Bounded review of a saved combined stop; no devices."""
import json,hashlib
from pathlib import Path
from collections import Counter
ROOT=Path(__file__).resolve().parent
RUN=ROOT/'pitch_slow_yaw_20261029_004532_520638'
r=json.loads((RUN/'result.json').read_bytes())
out=RUN/'offline_review';out.mkdir(exist_ok=True)
keys=['status','error','failed_phase','read_failures','read_recovery_confirmation_events',
    'slow_complete_sync_reads','precise_transport_clock_policy','sync_receive_window_policy',
    'serial_link_policy','final_off_reference_valid','final_off_endpoint_drift_ticks',
    'on_to_off_position_change_ticks','all_torque_off_verified','settings_restored','cube_present']
review={k:r[k] for k in keys if k in r}
review['last_samples']=[{k:s[k] for k in ('phase','t','read_ms','goal','actual','current_ma','voltage_v','tracking_rad') if k in s} for s in r['samples'][-6:]]
review['read_duration_ms_distribution']=dict(Counter(s.get('read_ms') for s in r['samples']))
review['result_sha256']=hashlib.sha256((RUN/'result.json').read_bytes()).hexdigest()
(out/'stop_diagnostic.json').write_text(json.dumps(review,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in review.items() if k not in ('last_samples','read_duration_ms_distribution')},ensure_ascii=False))
print('LAST READS',json.dumps([dict(phase=s['phase'],t=s['t'],read_ms=s.get('read_ms')) for s in r['samples'][-12:]]))
print('READ TIMING',json.dumps(sorted(review['read_duration_ms_distribution'].items(),key=lambda p:p[1],reverse=True)[:15]))
