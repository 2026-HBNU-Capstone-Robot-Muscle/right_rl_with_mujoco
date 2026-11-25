"""One-micrometre reporting resolution for known support collision pairs.

Geometry, contact forces and real motor criteria are unchanged. A previous
case retained a 0.0005703 mm support-edge overlap at the escape deadline.
This resolution is not a claim that the actual book/hand geometry is calibrated.
"""
import math
from book_empty_preparation_plan import ALLOWED_SUPPORT_PAIRS
ZERO_MM=.001
def classify(pairs):
    return {pair:depth for pair,depth in pairs.items() if not (pair in ALLOWED_SUPPORT_PAIRS and 0<=depth<=ZERO_MM)}
def accepted_discard(pair,depth):
    return pair in ALLOWED_SUPPORT_PAIRS and isinstance(depth,(int,float)) and math.isfinite(depth) and 0<=depth<=ZERO_MM
def metadata():
    return dict(version='known_support_pair_one_micrometre_resolution_v1_20261026',resolution_mm=ZERO_MM,
        pairs=sorted(ALLOWED_SUPPORT_PAIRS),previous_observed_edge_overlap_mm=.0005702054805453449,
        geometry_or_contact_forces_changed=False,hardware_health_limits_changed=False,
        escape_deadline_extended=False,physical_geometry_calibration_claimed=False)
