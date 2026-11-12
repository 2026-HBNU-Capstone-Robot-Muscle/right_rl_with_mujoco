"""Count repeated commands without claiming they can safely be removed."""
import json
import numpy as np
from dynamic_common_grip_search_20261105 import ROOT
z=np.load(ROOT/'continuous_handoff/candidate.npz');d=np.max(abs(np.diff(np.vstack([z['start'],z['goals']]),axis=0)),axis=1);runs=[];n=0
for value in d:
    if value==0:n+=1
    elif n:runs.append(n);n=0
if n:runs.append(n)
print(json.dumps(dict(goals=len(d),zero_increment=int(sum(d==0)),one_tick=int(sum(d==1)),two_ticks=int(sum(d==2)),constant_runs=sorted(runs,reverse=True),minimum_count_from_total_variation=int(sum(np.ceil(d/2))),retiming_not_verified=True)))
