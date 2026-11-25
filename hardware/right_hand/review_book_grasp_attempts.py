"""Read saved grasp attempts only. No motor or camera interface."""
import json
from pathlib import Path
from collections import Counter
ROOT=Path(__file__).resolve().parent
records=[]
for folder in sorted(ROOT.glob('book_cube_grasp_trial_*')):
    path=folder/'result.json'
    if not path.is_file():continue
    r=json.loads(path.read_bytes());samples=r.get('samples',[])
    row=dict(folder=folder.name,**{k:r.get(k) for k in ('status','error','failed_phase',
        'all_torque_off_verified','settings_restored','current_start_screen_accepted',
        'thumb_release_measured_verified','thumb_release_measurement','cube_placement_confirmed',
        'cube_removal_confirmed','camera_recording','fixed_hold_fault')})
    row.update(motor_motion_started='enabled_at' in r,write_events=len(r.get('write_events',[])),
        samples=len(samples),phase_counts=dict(Counter(s['phase'] for s in samples)))
    if samples:
        row.update(minimum_voltage_v=min(min(s['voltage_v']) for s in samples),
            peak_current_ma=max(max(abs(v) for v in s['current_ma']) for s in samples),
            max_tracking_rad=max(max(s['tracking_rad']) for s in samples),
            last_sample=samples[-1])
    records.append(row)
out=ROOT/'right_hand_rotation_20261024/book20_current_review/grasp_attempt_review.json'
out.write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
for row in records:print(json.dumps(row,ensure_ascii=False))
