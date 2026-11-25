"""Review the user's saved slow run offline. No device access."""
import json
from collections import Counter
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
RUN=ROOT/'slow_forward_placement_20261026_045454_900706'
OUT=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/slow_run_0454'

def main():
    r=json.loads((RUN/'result.json').read_bytes())
    s=r.get('samples',[]); w=r.get('write_events',[])
    keys=('status','error','failed_phase','all_torque_off_verified','settings_restored',
          'current_start_screen_accepted','cube_placement_confirmed','cube_removal_confirmed',
          'physical_flip_verified','last_issued_command','fixed_hold_fault','progress_fault',
          'camera_recording','camera_off_adjustment_waits','final_off_endpoint_drift_ticks')
    out={k:r.get(k) for k in keys}
    out.update(folder=RUN.name,samples=len(s),writes=len(w),result_keys=list(r),
               phases=dict(Counter(x.get('phase') for x in s)),
               write_phases=dict(Counter(x.get('phase') for x in w)))
    if s:
        out.update(first_sample=s[0],last_samples=s[-2:],
                   max_tracking_rad=max(max(x['tracking_rad']) for x in s),
                   peak_current_ma=int(max(np.max(abs(np.asarray(x['current_ma']))) for x in s)),
                   min_voltage=min(min(x['voltage_v']) for x in s),
                   low_voltage_samples=[dict(phase=x.get('phase'),t=x.get('t'),voltage_v=x['voltage_v']) for x in s if min(x['voltage_v'])<3.7])
    out['physical_rotation_from_encoders_verified']=False
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/'run_review.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
    printable={k:v for k,v in out.items() if k not in ('first_sample','result_keys')}
    print(json.dumps(printable,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
