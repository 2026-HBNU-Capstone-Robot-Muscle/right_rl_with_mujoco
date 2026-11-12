"""Bounded newest-run report. Saved files only; no devices."""
import json
from pathlib import Path
from collections import Counter
R=Path(__file__).resolve().parent
def main():
    for p in sorted(R.glob('forward_hold_observe_*/result.json'),key=lambda p:p.stat().st_mtime)[-2:]:
        d=json.loads(p.read_bytes());s=d.get('samples',[])
        keys=('timestamp','status','phase','error','failed_phase','hardware_authorized','all_torque_off_verified','settings_restored','cube_placement_confirmed','cube_support_confirmed_before_off','cube_removal_confirmed','cube_present','last_issued_phase','fixed_hold_seconds','final_off_reference_valid')
        row=dict(folder=p.parent.name,summary={k:d.get(k) for k in keys},samples=len(s),phases=dict(Counter(v['phase'] for v in s)),camera=d.get('camera_recording'),files=[f.name for f in p.parent.iterdir() if f.is_file()])
        if s:
            row['health']=dict(min_voltage_v=min(min(v['voltage_v']) for v in s),max_current_ma=max(max(abs(a) for a in v['current_ma']) for v in s),max_tracking_rad=max(max(v['tracking_rad']) for v in s),max_temperature_c=max(max(v['temperature_c']) for v in s),last_t=s[-1]['t'])
        row['read_failures']=d.get('read_failures',[])[-3:]
        row['other_faults']={k:v for k,v in d.items() if any(w in k for w in ('fault','primary_motion_error')) and k not in ('samples','write_events')}
        for k in ('placement_photo','grasp_observation_photo','rotation_observation_photo','fixed_hold_observation_photo'):
            if k in d:row[k]=d[k]
        print(json.dumps(row,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
