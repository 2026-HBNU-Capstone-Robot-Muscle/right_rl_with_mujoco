"""Inspect saved sample timings and actual local clock implementations."""
import json,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
folder=ROOT/'calibrated_quarter_trial_20261028_055850_634642';r=json.loads((folder/'result.json').read_bytes())
samples=r.get('samples',[])
small={k:v for k,v in r.items() if any(s in k for s in ('recover','read_','receive')) and k!='samples'}
small['tail_samples']=samples[-7:]
small['read_ms_p50_p95_max']=np.percentile([s['read_ms'] for s in samples],[50,95,100]).tolist()
small['clock_info']={name:vars(time.get_clock_info(name)) for name in ('monotonic','perf_counter')}
observations=[]
for _ in range(16):
    observations.append(dict(monotonic=time.monotonic(),perf_counter=time.perf_counter()));time.sleep(.001)
small['clock_observations']=observations
print(json.dumps(small))
(folder/'timing_review.json').write_bytes((json.dumps(small,indent=2)+'\n').encode())
