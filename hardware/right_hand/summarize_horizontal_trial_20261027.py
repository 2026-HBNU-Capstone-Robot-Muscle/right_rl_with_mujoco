"""Compact saved-trial review; never opens devices or prints all samples."""
import collections,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent
path=(ROOT/sys.argv[1] if len(sys.argv)>1 else ROOT/'leap_horizontal_quarter_trial_result.json')
r=json.loads(path.read_text(encoding='utf-8'));samples=r.get('samples',[])
keys=('timestamp','status','phase','failed_phase','error','stop_reason','all_torque_off_verified',
      'settings_restored','plan_version','request_file','current_start_screen','current_start_screen_accepted',
      'camera_recording','recording_folder','folder','voltage_events','goal_readback_events','read_failures')
out={k:r[k] for k in keys if k in r};out.update(sample_count=len(samples),
    phase_samples=dict(collections.Counter(v.get('phase') for v in samples)),
    last_samples=samples[-2:],top_keys=list(r))
for key in ('voltage_events','goal_readback_events','read_failures'):
    if key in out:out[key]=out[key][-2:]
print(json.dumps(out,ensure_ascii=False))
