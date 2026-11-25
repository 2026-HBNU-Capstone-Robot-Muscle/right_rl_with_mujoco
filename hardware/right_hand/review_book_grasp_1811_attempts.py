"""Read user paste and three saved attempts; no device access."""
import json
from collections import Counter
from pathlib import Path
root=Path(__file__).resolve().parent
paste=Path('C:/Users/김준서/.codex/attachments/786edbb2-5bc2-47fa-b39c-b9b4bc944291/Pasted text.txt')
print(paste.read_text(encoding='utf-8-sig'))
records=[]
for name in ('book_cube_grasp_trial_20261024_181129_687478',
             'book_cube_grasp_trial_20261024_181338_435214',
             'book_cube_grasp_trial_20261024_181424_938986'):
    folder=root/name;r=json.loads((folder/'result.json').read_bytes())
    samples=r.get('samples',[])
    row=dict(folder=name,**{k:r.get(k) for k in ('status','phase','error',
        'all_torque_off_verified','settings_restored','ring_release_measured_verified',
        'thumb_release_measured_verified','cube_placement_confirmed','cube_removal_confirmed',
        'fixed_hold_fault','preflight_state','fresh_off_state','final_on_state','post_off_state')})
    row.update(samples=len(samples),phase_counts=dict(Counter(s['phase'] for s in samples)),
        write_events=len(r.get('write_events',[])),result_keys=list(r))
    if samples:
        row.update(last_sample=samples[-1],minimum_voltage_v=min(min(s['voltage_v']) for s in samples),
            peak_current_ma=max(max(abs(v) for v in s['current_ma']) for s in samples),
            max_tracking_rad=max(max(s['tracking_rad']) for s in samples))
    camera=r.get('camera_recording',{})
    row['camera']={k:v for k,v in camera.items() if k not in ('frames','frame_index','samples','observations')}
    for file in ('current_start_screen.log','fresh_off_request.json'):
        if (folder/file).exists():
            data=(folder/file).read_text(encoding='utf-8')
            row[file]=data if file.endswith('.log') else json.loads(data)
    records.append(row)
out=root/'right_hand_rotation_20261024/book20_current_review/grasp_1811_review.json'
out.write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
for r in records:print(json.dumps(r,ensure_ascii=False))
