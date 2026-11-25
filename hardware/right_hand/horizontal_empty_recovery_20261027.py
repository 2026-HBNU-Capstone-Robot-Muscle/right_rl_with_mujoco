"""Pure transformation: raise one EMPTY root at a time, same endpoints."""
import numpy as np
from new_pair_grasp_plan_20261027 import ROOTS,OPEN
from fresh_empty_grasp_plan_20261027 import interpolate
RING_CLEAR='stage_0_ring_side_clearance'

def needs_ring_clearance(raw):
    a=np.asarray(raw)
    return bool(a.shape==(16,) and np.all(a[[7,11,15]]>=1980) and a[13]<2400 and a[8]>2900)

POLICY=dict(version='horizontal_ring_clearance_trimmed_roots_v5_20261027',motor_order=[8,12,16],
    initial_parallel_clearance_goal_ticks=1980,maximum_simultaneously_commanded_roots_after_clearance=1,average_root_step_ticks=1.5,
    pre_open_root_ceiling_ticks=2064,open_endpoint_unchanged=True,loaded_path_unchanged=True,
    conditional_ring_side_clearance_ticks=96,
    sample_period_seconds=.075,maximum_goal_step_ticks=2,
    current_and_pid_unchanged=True,voltage_thresholds_unchanged=True)


def serial_roots(raw,prefix):
    indices=[i for i,(phase,_) in enumerate(prefix) if phase==ROOTS]
    if not indices:return prefix
    first,last=indices[0],indices[-1]
    if indices!=list(range(first,last+1)):raise ValueError('Root recovery phases are not contiguous')
    current=(prefix[first-1][1] if first else raw).copy();target=prefix[last][1].copy()
    if any(current[i]!=target[i] for i in range(16) if i not in (7,11,15)):
        raise ValueError('Root segment changed a non-root goal')
    # The old index root target2150 overshot its final OPEN2048 by102ticks.
    # Only support clearance is needed before the existing thumb/open move.
    # Keep already higher fresh roots; do not introduce a backward recovery.
    for axis in (7,11):target[axis]=max(int(current[axis]),min(int(target[axis]),2064))
    leading=prefix[:first]
    if needs_ring_clearance(raw):
        side=current.copy();side[13]+=96
        leading=leading+[(RING_CLEAR,q) for q in interpolate(current,side)]
        current=side;target[13]=side[13]
    # The existing support-escape deadline still applies. Clear the first
    # small support contact together, before serializing the larger travel.
    clearance=current.copy()
    for axis in (7,11,15):clearance[axis]=min(int(target[axis]),max(int(current[axis]),1980))
    roots=[(ROOTS,q) for q in interpolate(current,clearance,1.5)];current=clearance
    for axis in (7,11,15):
        end=current.copy();end[axis]=target[axis]
        roots.extend((ROOTS,q) for q in interpolate(current,end,1.5));current=end
    if not np.array_equal(current,target):raise ValueError('Root endpoint mismatch')
    opening=[i for i,(phase,_) in enumerate(prefix) if phase==OPEN]
    if not opening or opening[0]!=last+1 or opening!=list(range(opening[0],opening[-1]+1)):
        raise ValueError('Expected the reviewed direct OPEN segment after root clearance')
    opened=prefix[opening[-1]][1].copy()
    # Rebuild the approach from the new root endpoint, so there is no jump
    # back to the previous2150-tick start. The OPEN endpoint stays identical.
    return leading+roots+[(OPEN,q) for q in interpolate(current,opened)]+prefix[opening[-1]+1:]
