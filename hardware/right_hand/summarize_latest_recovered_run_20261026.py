"""Read the latest saved recovered-placement result; no device access."""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
def main():
    p=sorted(v for v in ROOT.glob('recovered_forward_placement_*') if v.is_dir())[-1]
    r=json.loads((p/'result.json').read_bytes());s=r.get('samples',[])
    keys=('status','error','failed_phase','all_torque_off_verified','settings_restored','current_start_screen_accepted','cube_placement_confirmed','cube_removal_confirmed','physical_flip_verified','last_issued_command','fixed_hold_fault','progress_fault','camera_recording','camera_off_adjustment_waits','final_off_endpoint_drift_ticks')
    a={k:r.get(k) for k in keys};a.update(folder=p.name,samples=len(s),writes=len(r.get('write_events',[])))
    if s:
        a['last_samples']=s[-2:];a['max_tracking_rad']=max(max(v['tracking_rad']) for v in s)
        a['peak_current_ma']=int(max(np.max(abs(np.array(v['current_ma']))) for v in s))
        a['min_voltage']=min(min(v['voltage_v']) for v in s)
    print(json.dumps(a,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
