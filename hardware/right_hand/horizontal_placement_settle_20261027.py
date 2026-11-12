"""Require ten fresh stable OPEN samples; narrowly admit measured ID12 sag.

All other joints retain64 ticks. ID12 gets72 ticks, below the existing0.12rad
model tracking bound. No goal, gain, current, or hardware health limit changes.
"""
import numpy as np
from loaded_quarter_reentry_20261025 import limits
ERROR_LIMITS=tuple(72 if motor==12 else 64 for motor in range(1,17))
POLICY=dict(version='measured_stable_open_ID12_72_v1_20261027',sample_count=10,
    maximum_position_span_ticks=2,error_limits_by_motor_id_1_to_16=list(ERROR_LIMITS),
    exact_goal_readback_required=True,native_positions_required=True,
    goal_writes_or_resends=0,hardware_health_limits_unchanged=True)

def confirm(samples,command,report):
    event=dict(policy=dict(POLICY),status='checking');report['placement_settle_confirmation']=event
    try:
        target=np.asarray(command);lo,hi=limits()
        if target.shape!=(16,) or len(samples)!=10:raise RuntimeError('Placement needs10 complete fresh readings')
        positions=[];indices=[]
        for sample in samples:
            q=np.asarray(sample['position']);g=np.asarray(sample['goal']);index=sample['sample_index']
            if q.shape!=(16,) or g.shape!=(16,) or not np.isfinite(q).all() or np.any(q!=np.rint(q)):
                raise RuntimeError('Invalid placement encoder reading')
            if not isinstance(index,int) or isinstance(index,bool) or index<0 or indices and index<=indices[-1]:
                raise RuntimeError('Placement readings are missing or repeated')
            if not np.array_equal(g,target):raise RuntimeError('Placement goal readback changed; no loaded target advance')
            if np.any(q<lo) or np.any(q>hi):raise RuntimeError('Placement posture exceeds native positions')
            positions.append(q.copy());indices.append(index)
        values=np.asarray(positions);span=np.ptp(values,axis=0);error=np.max(abs(values-target),axis=0)
        event.update(position_span_ticks=span.tolist(),maximum_error_ticks_by_motor=error.tolist(),sample_indices=indices)
        if np.any(span>2):raise RuntimeError('Open preparation is still moving; no loaded target advance')
        bad=np.flatnonzero(error>np.asarray(ERROR_LIMITS))
        if len(bad):
            i=int(bad[0]);raise RuntimeError(f'Open placement ID{i+1} error{int(error[i])}ticks exceeds{ERROR_LIMITS[i]}; no loaded target advance')
        event['status']='confirmed_stable';return event
    except (Exception,KeyboardInterrupt) as exc:
        event.update(status='stopped',error=str(exc))
        raise
