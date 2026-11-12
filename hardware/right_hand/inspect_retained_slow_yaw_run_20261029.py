"""Bounded saved-file inspection only; no device or controller access."""
import json,hashlib,sys
from pathlib import Path
from collections import Counter
ROOT=Path(__file__).resolve().parent
folder=ROOT/(sys.argv[1] if len(sys.argv)>1 else 'retained_slow_yaw_20261029_012108_810204')
r=json.loads((folder/'result.json').read_bytes())
keys=('timestamp','status','error','failed_phase','plan_version','all_torque_off_verified','settings_restored',
    'cube_placement_confirmed','cube_removal_confirmed','cube_present','placement_orientation',
    'read_failures','read_recovery_confirmation_events','read_resynchronization_policy','last_issued_phase',
    'camera_worker_stopped','camera_recording','placement_photo','grasp_observation_photo','rotation_observation_photo',
    'opened_before_remove_photo','current_start_screen_accepted')
out={k:r[k] for k in keys if k in r}
samples=r.get('samples',[])
out['phase_counts']=dict(Counter(s['phase'] for s in samples));out['sample_count']=len(samples)
if samples:
    out['measured']=dict(min_voltage_v=min(min(s['voltage_v']) for s in samples),
        peak_abs_current_ma=max(max(abs(v) for v in s['current_ma']) for s in samples),
        max_tracking_rad=max(max(abs(v) for v in s['tracking_rad']) for s in samples))
    out['last_sample']={k:samples[-1][k] for k in ('phase','t','goal','actual','tracking_rad')}
frames=folder/'camera/frames.json'
if frames.exists():
    c=json.loads(frames.read_bytes());rows=c['frames'];phases={}
    for i,f in enumerate(rows):phases.setdefault(f['phase'],[]).append(i)
    out['camera']={k:v for k,v in c.items() if k not in ('frames','timestamps')}
    out['camera_phase_indices']={k:dict(first=v[0],middle=v[len(v)//2],last=v[-1],count=len(v)) for k,v in phases.items()}
out['named_images']=[p.name for p in folder.glob('*.jpg')]
out['result_sha256']=hashlib.sha256((folder/'result.json').read_bytes()).hexdigest()
dest=folder/'offline_review';dest.mkdir(exist_ok=True)
(dest/'saved_summary.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(out,ensure_ascii=False))
