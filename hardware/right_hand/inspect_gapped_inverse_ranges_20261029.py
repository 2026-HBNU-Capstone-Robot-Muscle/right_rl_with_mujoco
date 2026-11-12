"""Report actual goal-range discrepancies; no device access."""
import json
import numpy as np
from gapped_inverse_plan_20261029 import GappedInversePlan,REQUEST
from loaded_quarter_reentry_20261025 import limits
p=GappedInversePlan(REQUEST);start=p.source_position();rows=p.plan(start)
lo,hi=limits();found=[]
for i,(phase,q) in enumerate([('initial_OFF',start)]+rows):
    bad=np.flatnonzero((q<lo)|(q>hi))
    for j in bad:
        found.append(dict(sample=i,phase=phase,id=int(j+1),value=int(q[j]),lower=float(lo[j]),upper=float(hi[j])))
print(json.dumps(dict(start=start.tolist(),violations=len(found),first=found[:10],last=found[-5:]),ensure_ascii=False))
