"""Compact saved-attempt review. Reads files only; no device access."""
import json
from pathlib import Path
import numpy as np
from book_empty_preparation_plan import reference_ticks

root=Path(__file__).resolve().parent
reference=reference_ticks();records=[]
for folder in sorted(root.glob('book_prepare_empty_20261024_*')):
    path=folder/'result.json'
    if not path.is_file():continue
    r=json.loads(path.read_bytes());s=r.get('preflight_state');delta=[]
    if s:
        raw=np.array(s['position'])
        delta=[dict(id=i+1,reference=int(reference[i]),actual=int(raw[i]),difference_ticks=int(raw[i]-reference[i]))
            for i in range(16) if abs(raw[i]-reference[i])>4]
    row=dict(folder=folder.name,status=r['status'],error=r.get('error'),failed_phase=r.get('failed_phase'),
        torque_off_verified=r.get('all_torque_off_verified'),settings_restored=r.get('settings_restored'),
        settings_writes_started='initial' in r,motor_motion_started='enabled_at' in r,
        settings_restoration_not_needed='initial' not in r,samples=len(r.get('samples',[])),
        camera_recording=r.get('camera_recording'),pose_differences=delta,
        minimum_voltage_v=min(s['voltage'])/10 if s else None)
    records.append(row)
out=root/'right_hand_rotation_20261024/book20_current_review/user_attempt_review.json'
out.write_text(json.dumps(dict(attempts=records,motor_motion_started=any(v['motor_motion_started'] for v in records)),indent=2),encoding='utf-8')
for row in records:
    print(json.dumps({k:v for k,v in row.items() if k!='camera_recording'},ensure_ascii=False))
