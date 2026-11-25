"""Process-local limits bound to the user's freshly held all-ON reference."""
from contextlib import contextmanager
from pathlib import Path
import hashlib,json
import numpy as np
import leap_measured_grip_hold_20261117 as pose

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261118/grip_to_rotation/retaught_093413'
REFERENCE=BASE/'reference.json'
ORIGINAL_BOUNDS=pose.bounds
_ACTIVE_BOUNDS=None

def reference():
    ref=json.loads(REFERENCE.read_bytes());source=ROOT/ref['source']
    if hashlib.sha256(source.read_bytes()).hexdigest()!=ref['source_sha256']:raise RuntimeError('Retaught pose source changed')
    raw=json.loads(source.read_bytes());samples=raw['snapshots']
    if raw['status']!='readonly_observation_complete' or raw['max_two_read_difference_ticks']>1:
        raise RuntimeError('Retaught pose was not stable')
    for s in samples:
        if s['torque']!=[1]*16 or any(s['error']):raise RuntimeError('Retaught pose was not all ON and error-free')
    goal=pose.encoder_vector(samples[-1]['goal'])
    if not all(s['goal']==goal.tolist() for s in samples) or ref['holding_goal_ticks']!=goal.tolist():
        raise RuntimeError('Retaught holding goals changed')
    if max(np.max(abs(np.asarray(s['position'])-goal)) for s in samples)>4:
        raise RuntimeError('Retaught position did not follow its holding goal')
    return ref,goal

def scoped_bounds():
    if _ACTIVE_BOUNDS is None:raise RuntimeError('Recorded target scope not active')
    return tuple(a.copy() for a in _ACTIVE_BOUNDS)

@contextmanager
def limits():
    # Exact measured endpoints only, no margin, EEPROM edit or global source
    # change. Shared current/PID/tracking/progress/read/timeout checks stay on.
    global _ACTIVE_BOUNDS
    _,target=reference();previous=pose.bounds
    if previous is not ORIGINAL_BOUNDS:raise RuntimeError('Unexpected pose limit override')
    admission,high,low=ORIGINAL_BOUNDS()
    _ACTIVE_BOUNDS=(np.minimum(admission,target),np.maximum(high,target),np.minimum(low,target))
    pose.bounds=scoped_bounds
    try:yield
    finally:pose.bounds=previous;_ACTIVE_BOUNDS=None
