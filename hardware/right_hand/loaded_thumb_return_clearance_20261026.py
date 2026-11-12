"""Lift thumb CMC by two ticks during EMPTY return; all endpoints preserved."""
import numpy as np
from loaded_quarter_trial_plan_20261025 import empty_return_goals as original_goals
EXTRA_CMC_TICKS=2
def goals(anchor,extra=EXTRA_CMC_TICKS):
    original=original_goals(anchor);points=original.copy()
    distance=np.minimum(np.arange(len(points)),np.arange(len(points))[::-1])
    points[:,3]-=np.minimum(extra,distance)
    out=[points[0].copy()]
    for a,b in zip(points,points[1:]):
        count=max(1,int(np.ceil(np.max(abs(b-a))/2)))
        out.extend(np.rint(a+(b-a)*k/count).astype(np.int64) for k in range(1,count+1))
    return np.asarray(out,dtype=np.int64)
def metadata():
    return dict(scope='EMPTY return before PLACE only',motor_id=4,
                maximum_additional_CMC_lift_ticks=EXTRA_CMC_TICKS,
                same_start_and_endpoints=True,maximum_goal_step_ticks=2,
                loaded_goals_changed=False,hardware_health_limits_changed=False,
                contact_geometry_or_contact_thresholds_changed=False)
