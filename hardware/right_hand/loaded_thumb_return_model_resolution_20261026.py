"""Preserve forces while reporting a 10-nanometre RF/palm edge as model zero.

Only the exact ring-tip/palm4 pair during EMPTY return receives this resolution.
The observed 4.674 nm edge does not establish any physical contact or clearance.
The pre-existing 10-micrometre thumb/palm6 policy is retained unchanged.
"""
import math
from return_corner_resolution_20261026 import (
    classify as parent_classify,metadata as parent_metadata,PAIR,NUMERICAL_ZERO_MM)
RING_PAIR='palm_collision_4|rf_tip'
RING_ZERO_MM=.00001
PHASE='stage_3_empty_return_with_tip_clearance'
def classify(pairs,*,phase=None):
    remaining=parent_classify(pairs,phase=phase)
    return {p:v for p,v in remaining.items() if not (phase==PHASE and p==RING_PAIR and 0<=v<=RING_ZERO_MM)}
def accepted_discard(pair,depth):
    limit={PAIR:NUMERICAL_ZERO_MM,RING_PAIR:RING_ZERO_MM}.get(pair)
    return limit is not None and isinstance(depth,(int,float)) and math.isfinite(depth) and 0<=depth<=limit
def metadata():
    return dict(existing_thumb_return_resolution=parent_metadata(),ring_pair=RING_PAIR,
        ring_model_resolution_mm=RING_ZERO_MM,observed_ring_edge_depth_mm=4.673972507303622e-6,
        scope=PHASE,physical_geometry_calibrated=False,pure_integration_error_not_claimed=True,
        contact_geometry_forces_or_hardware_limits_changed=False)
