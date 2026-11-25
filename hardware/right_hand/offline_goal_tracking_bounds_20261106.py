"""Tighten offline goal increments using actual tracking; no device access."""
import numpy as np

def tracking_increment_bounds(lower,upper,continuous,actual,maximum_error_ticks):
    values=[np.asarray(v,float) for v in (lower,upper,continuous,actual)]
    if any(v.shape!=(16,) or not np.isfinite(v).all() for v in values) or not np.isfinite(maximum_error_ticks) or maximum_error_ticks<=0:raise ValueError('finite16-vectors and positive tracking margin required')
    lower,upper,continuous,actual=values
    if np.any(lower>upper):raise ValueError('empty native/increment interval')
    safe_lower=actual-maximum_error_ticks-continuous;safe_upper=actual+maximum_error_ticks-continuous;lo=np.maximum(lower,safe_lower);hi=np.minimum(upper,safe_upper);outside=lo>hi
    # If the actual joint has already moved beyond the tighter planning band,
    # take the largest legal retreat. The independent physical guard remains.
    retreat=np.where(safe_upper<lower,lower,upper)
    return np.where(outside,retreat,lo),np.where(outside,retreat,hi)
