"""Inspect the latest communication stop and cleanup, without devices."""
import json
from pathlib import Path
root=Path(__file__).resolve().parent
r=json.loads((root/'horizontal_quarter_trial_20261027_225914_101070/result.json').read_bytes())
out={k:r.get(k) for k in ('error','failed_phase','all_torque_off_verified','settings_restored','read_failures','read_recovery_confirmation_events')}
events=r.get('write_events',[]);out['last_write_events']=events[-8:]
s=r.get('samples',[]);out['min_voltage']=min(min(v['voltage_v']) for v in s);out['peak_current']=max(max(abs(x) for x in v['current_ma']) for v in s)
out['max_tracking']=max(max(v['tracking_rad']) for v in s)
print(json.dumps(out,ensure_ascii=False))
