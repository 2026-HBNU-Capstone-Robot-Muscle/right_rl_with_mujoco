"""Move a nearly extended ring PIP inward before lifting the empty roots.

Native limits, loaded grasp/rotation targets and drive settings are unchanged.
This is an explicitly model-screened EMPTY waypoint, not a range exception.
"""
import numpy as np
from new_pair_grasp_plan_20261027 import NewPairGraspPlan,ENTRY,ROOTS,OPEN,SETTLE,OPEN_PATH
from horizontal_empty_recovery_20261027 import serial_roots,RING_CLEAR
from loaded_quarter_reentry_20261025 import limits,validate_path
PHASE='stage_0_ring_pip_margin';PIP_INDEX=14;MARGIN_TICKS=16
POLICY=dict(version='empty_ring_PIP_inward_margin_v1_20261028',motor_id=15,minimum_native_lower_headroom_ticks=16,
    goal_step_ticks=2,only_before_empty_roots=True,no_range_extension=True,no_current_or_gain_increase=True,
    initial_support_scope='existing_source_pairs_only_nonincreasing_until_roots_clear',
    new_or_deeper_support_contact_allowed=False,later_support_threshold_mm=.15)
def interpolate(a,b):
    count=max(1,int(np.ceil(max(abs(b-a))/2)))
    return np.rint([a+(b-a)*i/count for i in range(1,count+1)]).astype(int)
def rows(plan,raw):
    a=plan.validate_start(raw)
    prefix=serial_roots(a,[(phase,q) for phase,q in NewPairGraspPlan.plan(plan,raw) if phase in (ENTRY,RING_CLEAR,ROOTS,OPEN,SETTLE)])
    low,_=limits();target=int(np.ceil(low[PIP_INDEX]))+MARGIN_TICKS
    if int(a[PIP_INDEX])>=target:return prefix
    begin=next(i for i,(phase,q) in enumerate(prefix) if phase==ROOTS)
    lead=prefix[:begin];start=lead[-1][1].copy() if lead else a.copy();margin=start.copy();margin[PIP_INDEX]=target
    result=lead+[(PHASE,q) for q in interpolate(start,margin)]
    for phase,q in prefix:
        if phase==ROOTS:
            q=q.copy();q[PIP_INDEX]=target;result.append((phase,q))
    opened=np.load(OPEN_PATH)['open_goal']
    result.extend((OPEN,q) for q in interpolate(result[-1][1],opened))
    result.extend((SETTLE,q.copy()) for phase,q in prefix if phase==SETTLE)
    validate_path(a,result,ENTRY)
    return result
def review(plan):
    a=plan.source_position();low,_=limits();target=int(np.ceil(low[PIP_INDEX]))+MARGIN_TICKS
    points=[q for phase,q in plan.plan(a) if phase==PHASE];required=a[PIP_INDEX]<target
    if bool(points)!=bool(required):raise RuntimeError('Missing/unexpected initial ring PIP margin')
    if points and (points[-1][PIP_INDEX]!=target or len(points)>16):raise RuntimeError('Unbounded initial ring PIP margin')
    return dict(policy=dict(POLICY),enabled=bool(points),source_ticks=int(a[PIP_INDEX]),target_ticks=target if points else int(a[PIP_INDEX]),
        maximum_command_travel_ticks=int(target-a[PIP_INDEX]) if points else 0)
