"""Fresh measured thumb clearance before the first root/preparation target."""
import numpy as np


def verify(state,source,threshold,report):
    actual=np.asarray(state['position']);source=np.asarray(source)
    details=dict(motor_id=1,measured_tip_ticks=int(actual[0]),required_tip_ticks_at_most=int(threshold),
        other_thumb_source_difference_ticks=(actual[1:4]-source[1:4]).tolist(),
        maximum_other_thumb_source_difference_ticks=4,physically_registered_geometry=False)
    report['thumb_release_measurement']=details
    if actual[0]>threshold:
        raise RuntimeError(f'Thumb release not measured: ID1={actual[0]}, need<={threshold}; no root target advance')
    if np.any(abs(actual[1:4]-source[1:4])>4):
        raise RuntimeError('Other thumb joints left the screened release neighborhood; no root target advance')
    report['thumb_release_measured_verified']=True
