"""Initial-only envelope for the recorded OFF thumb resting on the palm.

This changes no geometry, forces, joint limits or measured health stops. It
applies only to the two observed thumb/palm surfaces at an admitted OFF start.
The full native replay validates each fresh start, including OFF settling.
Every replay must still
separate existing contacts monotonically and allow no contact afterwards.
"""
import hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
PAIR='palm_collision_7|th_tip'
ADDITIONAL_PAIR='palm_collision_6|th_tip'
RECORDED=np.array([2414,3192,2623,2744,2123,2103,1767,1874,2162,2046,1749,1887,2836,2207,2915,2867])
EVIDENCE=ROOT/'forward_cube_placement_20261026_031956_975491/fresh_off_request.json'

def metadata(raw):
    a=np.asarray(raw)
    if a.shape!=(16,) or not np.isfinite(a).all() or np.any(a!=np.rint(a)):
        raise ValueError('Expected16 finite integer OFF positions')
    # Pose/range admission happens in the plan before this policy is called.
    # Do not reject ordinary OFF settling because it left an old16tick box.
    active=True
    # Use the pre-existing overall4mm thumb/palm initial envelope for this
    # recorded rest pose. Its +/-4tick sensitivity contains3.36mm here;
    # the nominal saved posture contains1.97mm. The overall envelope does
    # not increase. All contacts must still clear before opening continues.
    return dict(revision='admitted_OFF_known_thumb_palm_surfaces_v4_20261026',
        active=active,pair=PAIR,initial_pair_limit_mm=4. if active else .5,
        additional_initial_pairs=[ADDITIONAL_PAIR] if active else [],
        recorded_pose_ticks=RECORDED.tolist(),start_admission_required=True,
        previous_pose_match_required=False,known_surfaces_only=True,
        sensitivity_offset_ticks=4,evidence=EVIDENCE.relative_to(ROOT).as_posix(),
        evidence_sha256=hashlib.sha256(EVIDENCE.read_bytes()).hexdigest(),
        source_is_saved_OFF_reading=True,physical_mesh_registration_verified=False,
        initial_release_only=True,static_monotonic_contact_separation_required=True,
        dynamic_no_contact_beyond_initial_envelope=True,
        new_contacts_forbidden=True,after_release_self_overlap_limit_mm=1e-6,
        geometry_and_contact_forces_changed=False,actual_health_stops_changed=False)
