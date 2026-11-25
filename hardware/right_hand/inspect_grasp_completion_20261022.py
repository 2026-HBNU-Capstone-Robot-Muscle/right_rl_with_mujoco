"""Read the completed empty approach; no devices or setting changes."""
import hashlib
import json
from collections import Counter
from pathlib import Path

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'grasp_approach_current_empty_20261022_020751_741219'
data=(FOLDER/'result.json').read_bytes();r=json.loads(data)
print('sha256',hashlib.sha256(data).hexdigest())
print('top keys',list(r))
print('sample keys',list(r['samples'][0]))
print('sample phases',dict(Counter(s['phase'] for s in r['samples'])))
print('write example',r['write_events'][0])
for phase in ('stage_3_ring_clearance','stage_4_grasp_approach'):
    group=[s for s in r['samples'] if s['phase']==phase]
    print(phase,'first',group[0],'last',group[-1])
print('review',json.loads((FOLDER/'review.json').read_bytes()))
