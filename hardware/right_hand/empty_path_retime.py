"""Integer midpoints for a 2x-time EMPTY diagnostic; no hardware imports."""
import numpy as np

def half_speed(start, goals):
    start=np.asarray(start);goals=np.asarray(goals)
    if start.shape!=(16,) or goals.ndim!=2 or goals.shape[1]!=16 or not len(goals):
        raise ValueError('Expected a start and nonempty 16-joint path')
    raw=np.vstack([start,goals])
    if not np.isfinite(raw).all() or np.any(raw!=np.rint(raw)) or np.any(raw<0) or np.any(raw>4095):
        raise ValueError('Invalid single-turn path')
    mid=np.rint((raw[:-1]+raw[1:])/2).astype(int)
    result=np.empty((2*len(goals),16),dtype=int)
    result[::2]=mid;result[1::2]=goals
    return result
