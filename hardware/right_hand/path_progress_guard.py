"""Detect immobility while goals/current rise; never relax existing health trips."""
import numpy as np

def check_progress(samples):
    if len(samples)<6:return
    rows=samples[-6:]
    if not .2<=rows[-1]['t']-rows[0]['t']<=.8:return
    actual=np.array([r['actual'] for r in rows]);goals=np.array([r['goal'] for r in rows]);current=np.array([r['current_ma'] for r in rows])
    stationary=np.ptp(actual,axis=0)<=2
    error=goals[-1]-actual[-1]
    # A retracting goal can reduce force while the position remains unchanged.
    # Only diagnose sustained target advancement AWAY from measured position.
    direction=np.sign(error)
    requested=(goals[-1]-goals[0])*direction>=16
    moving_away=np.all(np.diff(goals,axis=0)*direction>=0,axis=0)
    lag=np.abs(error)>=40
    loaded=np.abs(current[-1])>=80
    ids=np.flatnonzero(stationary&requested&moving_away&lag&loaded)
    if len(ids):
        i=int(ids[0])
        raise RuntimeError(f'ID{i+1} no measured progress across 6 path samples; actual={actual[-1,i]}, goal={goals[-1,i]}, current={current[-1,i]}mA; no target advance')
