"""Bounded inspection of the newest saved run; no devices or motor imports."""
import json
from pathlib import Path
R=Path(__file__).resolve().parent
def main():
    path=max(R.glob('forward_hold_observe_*/result.json'),key=lambda p:p.stat().st_mtime);d=json.loads(path.read_bytes())
    keys=('timestamp','status','phase','error','failed_phase','hardware_authorized','all_torque_off_verified','settings_restored','current_start_screen','current_start_screen_accepted','enabled_at','last_issued_phase','cube_placement_confirmed','cube_present','cube_removal_confirmed','final_off_reference_valid')
    out=dict(folder=path.parent.name,summary={k:d.get(k) for k in keys},sample_count=len(d.get('samples',[])),write_events=len(d.get('write_events',[])))
    out['last_samples']=[{k:s.get(k) for k in ('phase','t','goal','actual','current_ma','voltage_v','tracking_rad','torque','error','read_ms')} for s in d.get('samples',[])[-3:]]
    for k in ('camera_recording','camera_startup','goal_readback_failures','read_failures','off_failures','restore_failures','off_start_observations','final_off_snapshots'):
        if k in d:out[k]=d[k][-3:] if isinstance(d[k],list) else d[k]
    out['extra_fields']=[k for k in d if any(s in k for s in ('fail','stop','exception','recover','timing'))]
    print(json.dumps(out,ensure_ascii=False,indent=2))
    for name in ('current_start_screen.log','current_start_screen.json'):
        p=path.parent/name
        if p.exists():
            if p.suffix=='.log': print('SCREEN_LOG_TAIL',p.read_text(encoding='utf-8',errors='replace')[-6500:])
            else:
                v=json.loads(p.read_bytes());print('SCREEN_SUMMARY',json.dumps({k:v[k] for k in v if not isinstance(v[k],(list,dict))},ensure_ascii=False))
if __name__=='__main__':main()
