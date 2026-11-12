"""Compact saved trial review only; never opens hardware."""
from collections import Counter
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
for folder in sorted(ROOT.glob('grasped_yaw_trial_20261024_*'))[-4:]:
    path=folder/'result.json'
    if not path.exists():continue
    r=json.loads(path.read_bytes());samples=r.get('samples',[])
    print('TRIAL',folder.name)
    print(json.dumps({k:r.get(k) for k in ('status','error','failed_phase','all_torque_off_verified',
        'settings_restored','camera_exposure_restoration_verified','rotation_command_path_completed',
        'cube_placement_confirmed','cube_removal_confirmed','rotation_candidate',
        'rotation_evidence_cutoff_camera_monotonic')},ensure_ascii=False))
    print('PHASES',dict(Counter(x.get('phase') for x in samples)))
    if samples:
        print('HEALTH',dict(min_voltage=min(min(s['voltage_v']) for s in samples),
            peak_current=max(abs(v) for s in samples for v in s['current_ma']),
            max_tracking=max(max(s['tracking_rad']) for s in samples)))
    for key in ('read_failures','goal_readback_confirmation_events','goal_readback_mismatches',
                'read_recovery_confirmation_events'):
        events=r.get(key,[])
        print(key,len(events),events[-1:] if events else [])
    for filename in ('fresh_off_request.json','current_start_screen.json','camera/frames.json'):
        p=folder/filename
        if not p.exists():continue
        record=json.loads(p.read_bytes())
        if filename.endswith('screen.json'):print('SCREEN',record.get('accepted'))
        if filename.endswith('frames.json'):
            rows=record['frames']
            print('FRAMES',len(rows),dict(Counter(f.get('phase') for f in rows)))
    print('FILES',[p.name for p in folder.iterdir() if p.is_file()])
