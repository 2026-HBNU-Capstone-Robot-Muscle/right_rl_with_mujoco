import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
for folder in sorted(ROOT.glob('horizontal_quarter_trial_20261027_*'))[-2:]:
 r=json.loads((folder/'result.json').read_bytes());a=np.array(r['initial']['position']);rows=r['off_preload_snapshots'];diffs=[]
 for i,s in enumerate(rows):
  q=np.array(s['position']);goal=np.array(s['goal']);prior=np.array(rows[i-1]['position']) if i else a
  diffs.append(dict(read=i+1,position_from_original={str(j+1):int(v) for j,v in enumerate(q-a) if v},goal_from_original={str(j+1):int(v) for j,v in enumerate(goal-a) if v},position_change={str(j+1):int(v) for j,v in enumerate(q-prior) if v},goal_equals_position=bool(np.array_equal(q,goal)),torque_on=any(s['torque']),min_voltage_v=min(s['voltage'])/10))
 print(json.dumps(dict(folder=folder.name,preload_reads=diffs,off_start_observations=r.get('off_start_observations')),ensure_ascii=False))
