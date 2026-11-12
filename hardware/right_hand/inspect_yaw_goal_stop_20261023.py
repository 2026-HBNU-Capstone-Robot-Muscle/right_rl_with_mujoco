"""Compact stored readback-failure diagnosis; no device access."""
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
folder=ROOT/'grasped_yaw_trial_20261023_222816_628099'
r=json.loads((folder/'result.json').read_bytes())
for event in r.get('goal_readback_events',[]):
    print('GOAL EVENT',json.dumps(event,ensure_ascii=False,indent=2))
samples=r['samples'];rows=[s for s in samples if s['phase']=='stage_3_loaded_approach']
for s in rows[-8:]:
    print('TAIL',json.dumps(dict(t=s['t'],goal=s['goal'],readback=s['goal_readback'],
        goal_error_ticks=(np.array(s['goal_readback'])-s['goal']).tolist(),
        maximum_tracking=max(s['tracking_rad']),read_ms=s['read_ms']),indent=2))
print('WRITES',[e for e in r['write_events'] if e['address']==116][-5:])
