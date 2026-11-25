"""Compact saved-run inspection. Never opens devices."""
import json
from collections import Counter
from pathlib import Path

ROOT=Path(__file__).resolve().parent


def short(value):
    if isinstance(value,dict):
        return {k:(f'list[{len(v)}]' if isinstance(v,list) and len(v)>5 else
                   {a:b for a,b in v.items() if not isinstance(b,(dict,list))} if isinstance(v,dict) else v)
                for k,v in value.items()}
    return value


for folder in sorted(ROOT.glob('grasped_yaw_trial_*'))[-3:]:
    path=folder/'result.json'
    if not path.exists():continue
    r=json.loads(path.read_bytes());samples=r.get('samples',[])
    print('\nFOLDER',folder.name)
    print('summary',{k:r.get(k) for k in ('timestamp','status','error','failed_phase',
        'all_torque_off_verified','settings_restored','physical_rotation_verified',
        'rotation_command_path_completed','cube_placement_confirmed','cube_removal_confirmed')})
    print('keys',list(r))
    print('phases',dict(Counter(x.get('phase') for x in samples)))
    if samples:
        print('last_sample',samples[-1])
        print('health',dict(min_voltage_v=min(min(x['voltage_v']) for x in samples),
            max_current_ma=max(abs(v) for x in samples for v in x['current_ma']),
            max_tracking_rad=max(max(x['tracking_rad']) for x in samples)))
    for key in ('read_failures','goal_readback_confirmation_events','read_recovery_confirmation_events',
                'vision_recording','camera_startup','grasped_yaw_observation','rotation_evidence_cutoff_camera_monotonic',
                'final_off_snapshots','goal_readback_mismatches'):
        if key in r:print(key,short(r[key]) if key!='final_off_snapshots' else r[key][-1:])
    print('files',[p.name for p in folder.iterdir() if p.is_file()])
