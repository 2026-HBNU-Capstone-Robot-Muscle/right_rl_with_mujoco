"""Small EMPTY thumb advance along the existing OPEN route, before roots.

Targets/ranges and the eventual OPEN endpoint are unchanged. This module has
no motor or model API and provides no collision tolerance exceptions.
"""
import numpy as np
from new_pair_grasp_plan_20261027 import ENTRY,ROOTS,OPEN
from fresh_empty_grasp_plan_20261027 import interpolate
THUMB_CLEAR='stage_0_thumb_book_clearance'
POLICY=dict(version='thumb_advance_existing_open_route_v1_20261027',
    maximum_thumb_advance_ticks=16,motor_ids=[1,2,3,4],maximum_goal_step_ticks=2,
    activate_below_id2_and_id9_ticks=2304,open_endpoint_unchanged=True,loaded_path_unchanged=True,
    extra_collision_or_health_tolerance=False)

def needed(raw):
    a=np.asarray(raw)
    return bool(a.shape==(16,) and a[1]<2304 and a[8]<2304)

def clear_thumb(raw,rows,maximum_advance=16):
    if not needed(raw):return rows
    roots=[i for i,(phase,_) in enumerate(rows) if phase==ROOTS]
    opened=[i for i,(phase,_) in enumerate(rows) if phase==OPEN]
    if not roots or not opened:raise ValueError('Expected root and OPEN segments')
    first,last=roots[0],roots[-1];oa,ob=opened[0],opened[-1]
    if roots!=list(range(first,last+1)) or oa!=last+1 or opened!=list(range(oa,ob+1)):
        raise ValueError('Unexpected recovery phase ordering')
    start=(rows[first-1][1] if first else np.asarray(raw)).copy();end=rows[ob][1].copy()
    delta=end[:4]-start[:4];advance=start.copy();fraction=min(1.,maximum_advance/max(1,int(max(abs(delta)))))
    advance[:4]+=np.rint(delta*fraction).astype(int)
    prefix=rows[:first]+[(THUMB_CLEAR,q) for q in interpolate(start,advance)]
    for phase,q in rows[first:last+1]:
        value=q.copy();value[:4]=advance[:4];prefix.append((phase,value))
    return prefix+[(OPEN,q) for q in interpolate(prefix[-1][1],end)]+rows[ob+1:]
