"""Bounded empty diagnostic specification; no hardware imports."""
import numpy as np

PREFIX_COUNT=40
HOLD_SECONDS=1.0

def select_prefix(goals):
    goals=np.asarray(goals)
    if goals.ndim!=2 or goals.shape[1]!=16 or len(goals)<PREFIX_COUNT:
        raise ValueError('Incomplete diagnostic path')
    prefix=goals[:PREFIX_COUNT].copy()
    if not np.isfinite(prefix).all() or np.any(prefix!=np.rint(prefix)):
        raise ValueError('Noninteger diagnostic targets')
    if prefix[-1,11]!=3290 or np.max(prefix[:,11])!=3290:
        raise ValueError('ID12 hold target must be reviewed 3290 ticks')
    return prefix.astype(int)
