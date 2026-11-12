"""Compact read-only review of an existing trial; never opens devices."""
import argparse
import json
from collections import Counter
from pathlib import Path

ROOT=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('path');args=parser.parse_args()
path=Path(args.path)
if not path.is_absolute():path=ROOT/path
if path.is_dir():path=path/'result.json'
r=json.loads(path.read_bytes())
fields=('timestamp','status','phase','error','failed_phase','all_torque_off_verified','settings_restored',
        'cube_placement_confirmed','cube_removal_confirmed','physical_grasp_verified',
        'physical_rotation_verified','physical_flip_verified','policy_used',
        'placement_photo','grasp_observation_photo','camera_recording','current_start_screen',
        'final_off_endpoint_drift_ticks','timing_passed','all_observed_torque_off',
        'max_camera_poll_ms','max_read_ms','max_read_gap_ms','servo_write_commands_sent')
print(json.dumps({k:r.get(k) for k in fields},ensure_ascii=False,indent=2))
rows=r.get('samples',[])
print('PHASES',dict(Counter(s['phase'] for s in rows)))
if rows:
    metrics={}
    if all('voltage_v' in s for s in rows):metrics['min_voltage']=min(min(s['voltage_v']) for s in rows)
    if all('current_ma' in s for s in rows):metrics['peak_current']=max(max(abs(v) for v in s['current_ma']) for s in rows)
    if all('tracking_rad' in s for s in rows):metrics['max_tracking']=max(max(s['tracking_rad']) for s in rows)
    print('METRICS',metrics)
frame_file=path.parent/'camera/frames.json'
if frame_file.exists():
    frames=json.loads(frame_file.read_bytes())['frames']
    for phase in dict.fromkeys(f['phase'] for f in frames):
        group=[f for f in frames if f['phase']==phase]
        print('CAMERA_PHASE',phase,len(group),[(group[i]['image'],round(group[i]['camera_read_monotonic'],3))
            for i in sorted(set([0,len(group)//2,len(group)-1]))])
