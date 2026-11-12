"""Summarize archived empty-path telemetry without devices or writes to results."""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent

def main():
    paths=sorted(ROOT.glob('leap_sac_path_empty_2*.json'))[-4:]
    reviews=[]
    for path in paths:
        d=json.loads(path.read_text(encoding='utf-8'));samples=d.get('samples',[])
        row={k:d.get(k) for k in ('timestamp','status','error','failed_phase','all_torque_off_verified','settings_restored','path_commands_sent')}
        row['file']=path.name
        if samples:
            low=min(samples,key=lambda s:min(s['voltage_v']))
            row['lowest_voltage_sample']={k:low.get(k) for k in ('phase','t','voltage_v','current_ma')}
        trace=[s for s in samples if s['phase']=='reachable_empty_path']
        row['path_samples']=len(trace)
        row['tail']=[dict(t=s['t'],goal16=s['goal'][15],actual16=s['actual'][15],
                         error16=s['tracking_rad'][15],current16=s['current_ma'][15],voltage16=s['voltage_v'][15],
                         read_ms=s['read_ms']) for s in trace[-15:]]
        cycles=d.get('path_cycle_seconds',[])
        if cycles:row['cycle_ms']=dict(min=min(cycles)*1000,median=float(np.median(cycles)*1000),max=max(cycles)*1000)
        reviews.append(row)
    out=dict(reviews=reviews,motor_commands_sent=0)
    (ROOT/'sac_path_hardware_review_result.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps(out,indent=2))

if __name__=='__main__':main()
