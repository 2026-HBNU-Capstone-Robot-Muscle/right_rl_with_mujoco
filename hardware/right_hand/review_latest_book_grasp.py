"""Compact saved-trial and raw-frame review; no device access."""
import json
import sys
from pathlib import Path
from collections import Counter
root=Path(__file__).resolve().parent
folder=root/sys.argv[1] if len(sys.argv)>1 else sorted(root.glob('book_cube_grasp_trial_*'))[-1]
r=json.loads((folder/'result.json').read_bytes());samples=r.get('samples',[])
row=dict(folder=folder.name,**{k:r.get(k) for k in ('status','error','failed_phase',
    'all_torque_off_verified','settings_restored','cube_placement_confirmed','cube_removal_confirmed',
    'ring_release_measured_verified','thumb_release_measured_verified','camera_recording',
    'final_on_state','final_off_reference_valid','final_off_endpoint_drift_ticks')})
row.update(phase_counts=dict(Counter(s['phase'] for s in samples)),samples=len(samples),
    write_events=len(r.get('write_events',[])),result_keys=list(r))
if samples:
    row.update(min_voltage_v=min(min(s['voltage_v']) for s in samples),
        peak_current_ma=max(max(abs(v) for v in s['current_ma']) for s in samples),
        max_tracking_rad=max(max(s['tracking_rad']) for s in samples),last_sample=samples[-1])
frames_path=folder/'camera/frames.json';selected=[]
if frames_path.exists():
    frames=json.loads(frames_path.read_bytes())['frames']
    for phase in dict.fromkeys(f['phase'] for f in frames):
        group=[f for f in frames if f['phase']==phase]
        for tag,f in [('first',group[0]),('last',group[-1])]:
            selected.append(dict(phase=phase,which=tag,path=str(folder/'camera'/f['image']),
                camera_read_monotonic=f['camera_read_monotonic']))
row['selected_frames']=selected
out=folder/'offline_review.json';out.write_text(json.dumps(row,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(row,ensure_ascii=False))
