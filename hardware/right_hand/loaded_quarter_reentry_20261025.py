"""Native goals, with only the existing observed ID4 OFF re-entry exception."""
import json
from pathlib import Path
import numpy as np
from loaded_quarter_joint_limits_20261025 import configured_limits
ROOT=Path(__file__).resolve().parent
def limits():
    return configured_limits()
def cmc_native_min():return int(np.ceil(limits()[0][3]))
def release_target(raw,join):
    a=np.asarray(raw,dtype=np.int64);release=a.copy();lo,_=limits()
    for j in (0,14):release[j]=max(int(np.ceil(lo[j])),int(a[j])-64)
    if a[3]<1872:release[3]=max(cmc_native_min(),1872)
    roots=np.array([7,11,15]);release[roots]+=np.minimum(64,np.maximum(0,np.asarray(join)[roots]-a[roots]))
    return release
def validate_path(raw,rows,release_phase):
    a=np.asarray(raw);values=np.vstack([a]+[v for _,v in rows]);lo,hi=limits()
    delta=abs(np.diff(values,axis=0))
    if delta.max()>2:
        i,j=np.unravel_index(int(delta.argmax()),delta.shape)
        raise ValueError(f'ID{j+1} goal step {int(delta[i,j])} ticks in {rows[i][0]}; limit 2')
    other=np.arange(16)!=3;bad=np.argwhere(((values<lo)|(values>hi))&other)
    if len(bad):
        i,j=bad[0];raise ValueError(f'ID{j+1} {values[i,j]} ticks outside native range [{lo[j]:.1f},{hi[j]:.1f}]; no writes')
    if np.any(values[:,3]>hi[3]):raise ValueError('ID4 exceeds its original native upper bound')
    previous=int(a[3]);restored=previous>=lo[3]
    for phase,q in rows:
        current=int(q[3])
        if not restored:
            if phase!=release_phase or current<previous:raise ValueError('ID4 OFF re-entry must only move toward its original native range before any other phase')
            restored=current>=lo[3]
        elif current<lo[3]:raise ValueError('ID4 target left its original native range after re-entry')
        previous=current
    if not restored:raise ValueError('ID4 native range not restored by the initial release')
    return values
