"""Finite, empty-only ID12 return probe within the already exercised angles."""
import numpy as np

END_TICKS = 3130
STEPS = 32
DT = .05

def targets(command):
    raw=np.asarray(command)
    if raw.shape!=(16,) or not np.isfinite(raw).all() or not np.equal(raw,np.rint(raw)).all():
        raise ValueError('Backoff needs 16 integer goals')
    if raw[11]!=3290:raise ValueError('Backoff must start at the reviewed 3290 goal')
    result=np.repeat(raw.astype(int)[None],STEPS,axis=0)
    result[:,11]=np.rint(np.linspace(3290,END_TICKS,STEPS+1)[1:]).astype(int)
    return result
