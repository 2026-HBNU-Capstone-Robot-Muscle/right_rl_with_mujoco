"""Inspect archived timing/encoder evidence; never opens devices."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

parser=argparse.ArgumentParser();parser.add_argument('folder');args=parser.parse_args()
folder=Path(args.folder);r=json.loads((folder/'result.json').read_bytes());rows=r.get('samples',[])
phases={}
for phase in dict.fromkeys(x['phase'] for x in rows):
    samples=[x for x in rows if x['phase']==phase]
    times=np.array([x['t'] for x in samples]);reads=np.array([x['read_ms'] for x in samples])
    phases[phase]=dict(samples=len(samples),first_t=float(times[0]),last_t=float(times[-1]),
        interval_percentiles_s=np.percentile(np.diff(times),[0,50,90,99,100]).tolist() if len(times)>1 else [],
        read_percentiles_ms=np.percentile(reads,[0,50,90,99,100]).tolist())
last=rows[-1] if rows else None
proof=json.loads((folder/'current_start_screen.json').read_bytes())
report=dict(source_sha256=hashlib.sha256((folder/'result.json').read_bytes()).hexdigest(),
    status=r['status'],error=r.get('error'),failed_phase=r.get('failed_phase'),
    all_torque_off_verified=r.get('all_torque_off_verified'),settings_restored=r.get('settings_restored'),
    plan_version=r.get('plan_version'),sample_count=len(rows),phase_timing=phases,
    min_voltage=min(min(x['voltage_v']) for x in rows),peak_current=max(max(abs(v) for v in x['current_ma']) for x in rows),
    max_tracking=max(max(x['tracking_rad']) for x in rows),
    first_goal=rows[0]['goal'],last_sample=last,final_command=r.get('final_command'),
    last_command_error_ticks=(np.array(last['actual'])-np.array(r['final_command'])).tolist(),
    last_goal_is_final=last['goal']==r.get('final_command'),
    final_off_reference_valid=r.get('final_off_reference_valid'),
    final_off_position=r.get('final_off_snapshots',[{}])[-1].get('position'),
    on_to_off_change=r.get('on_to_off_position_change_ticks'),
    native_screen_accepted=proof['accepted'],
    write_event_count=len(r.get('write_events',[])))
frames=json.loads((folder/'camera/frames.json').read_bytes())
report['camera_frames']=len(frames['frames']);groups={}
for frame in frames['frames']:groups.setdefault(frame['phase'],[]).append(frame)
report['phase_images']={k:dict(count=len(v),first=v[0]['image'],last=v[-1]['image']) for k,v in groups.items()}
(folder/'timing_review.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
