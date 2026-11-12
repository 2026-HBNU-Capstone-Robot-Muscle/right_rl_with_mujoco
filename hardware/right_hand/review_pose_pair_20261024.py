"""Review latest archived photo/encoder pair. Never open devices."""
import hashlib
import json
from pathlib import Path
import numpy as np
from leap_pose_pair_readonly import validate_pair

ROOT=Path(__file__).resolve().parent
folder=sorted(ROOT.glob('pose_pair_readonly_20261024_*'))[-1]
source=folder/'result.json';blob=source.read_bytes();r=json.loads(blob)
summary=dict(source=str(source.relative_to(ROOT)),source_sha256=hashlib.sha256(blob).hexdigest(),
    status=r['status'],error=r.get('error'),servo_write_commands_sent=r['servo_write_commands_sent'],
    torque_changes_requested=r['torque_changes_requested'],image=r.get('image'),
    camera_read_monotonic=r.get('camera_read_monotonic'),snapshots=len(r.get('snapshots',[])))
if len(r.get('snapshots',[]))==2 and r['status']=='photo_and_readings_saved':
    first,last=r['snapshots'];check=validate_pair(first,last)
    actual=[np.array([next(m['position_ticks_signed'] for m in s['motors'] if m['id']==i) for i in range(1,17)]) for s in (first,last)]
    old=json.loads((ROOT/'bus_readonly_diagnose_20261024_043442_559037/result.json').read_bytes())
    previous=np.array(old['stream_samples'][-1]['position_ticks'])
    summary.update(pair_check=check,before_ticks=actual[0].tolist(),after_ticks=actual[1].tolist(),
        change_from_previous_OFF_read_ticks=(actual[0]-previous).tolist(),
        minimum_voltage_v=min(m['voltage_v'] for s in (first,last) for m in s['motors']),
        maximum_temperature_c=max(m['temperature_c'] for s in (first,last) for m in s['motors']),
        observed_watchdog_values=sorted({m['bus_watchdog_raw'] for s in (first,last) for m in s['motors']}),
        photo_after_before_read_end=r['camera_read_monotonic']>first['read_end_monotonic'],
        photo_before_after_read_start=r['camera_read_monotonic']<last['read_start_monotonic'])
summary.update(real_devices_opened_by_review=False,geometry_calibration_verified=False,physical_rotation_verified=False)
(folder/'paired_review.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(summary,ensure_ascii=False,indent=2))
print('PHOTO',folder/r.get('image','paired_photo.png'))
