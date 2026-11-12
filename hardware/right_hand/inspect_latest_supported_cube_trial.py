"""Compact inspection of the user's next saved run; never opens devices."""
import json
from collections import Counter
from pathlib import Path
ROOT=Path(__file__).resolve().parent
path=ROOT/'leap_cube_supported_rotation_trial_result.json'
if not path.exists():
    print('No supported cube trial result has been saved yet.')
    raise SystemExit(0)
r=json.loads(path.read_bytes())
fields=('timestamp','status','phase','error','failed_phase','all_torque_off_verified','settings_restored',
    'cube_placement_confirmed','cube_removal_confirmed','rotation_command_path_completed','physical_rotation_verified')
print(json.dumps({k:r.get(k) for k in fields},ensure_ascii=False,indent=2))
print('CAMERA',json.dumps(r.get('camera_recording'),ensure_ascii=False))
rows=r.get('samples',[])
print('PHASES',dict(Counter(s['phase'] for s in rows)))
if rows:
    print('MIN_VOLTAGE_V',min(min(s['voltage_v']) for s in rows))
    print('PEAK_CURRENT_MA',max(max(abs(v) for v in s['current_ma']) for s in rows))
    print('MAX_TRACKING_RAD',max(max(s['tracking_rad']) for s in rows))
    print('LAST_SAMPLE',json.dumps(rows[-1],ensure_ascii=False)[:2200])
for key in ('rotation_observation_photo','placement_photo','fixed_hold_fault','final_off_endpoint_drift_ticks'):
    if key in r:print(key,json.dumps(r[key],ensure_ascii=False))
