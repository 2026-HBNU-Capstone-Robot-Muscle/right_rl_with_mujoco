"""Initial-only known palm face, after the user confirmed actual thumb contact.

The same per-pair monotonic separation and no-new-contact checks still apply.
Geometry, contact forces, native ranges and hardware health checks are retained.
"""
import hashlib,json
from pathlib import Path
import numpy as np
from interrupted_thumb_contact_policy_20261026 import metadata as parent_metadata
ROOT=Path(__file__).resolve().parent
EVIDENCE=ROOT/'confirmed_thumb_palm_contact_20261026.json'
PREFIX=ROOT/'confirmed_thumb_escape_prefix_20261026.json'
PAIR='palm_body|th_tip'
MAX_INITIAL_MM=4.
REFERENCE=np.array([2339,3185,2623,2822,2123,2107,1768,1876,2157,2058,1752,1890,2783,2243,2821,2769],dtype=int)
def active(raw):return bool(np.max(abs(np.asarray(raw)-REFERENCE))<=32)
def prefix(raw):
    if not active(raw):return []
    saved=json.loads(PREFIX.read_bytes());a=np.asarray(raw,dtype=int);rows=[]
    for delta in saved['thumb_delta_waypoints_ticks']:
        q=a.copy();q[:4]+=np.asarray(delta,dtype=int);rows.append(q)
    return rows
def is_palm_face_pair(pair):
    a,sep,b=pair.partition('|')
    return bool(sep and b=='th_tip' and a.startswith('palm_collision_') and a[len('palm_collision_'):].isdigit())
def classify_initial_palm(pairs,*,enabled,initial_release=False,phase=None):
    if not enabled or not (initial_release or phase=='initial_hold'):return dict(pairs)
    result={pair:depth for pair,depth in pairs.items() if not is_palm_face_pair(pair)}
    depths=[depth for pair,depth in pairs.items() if is_palm_face_pair(pair)]
    if depths:result[PAIR]=max(depths)
    return result
def verify_palm_body(model):
    names=[model.geom(i).name for i in range(model.ngeom)
        if is_palm_face_pair(str(model.geom(i).name)+'|th_tip')]
    if not {'palm_collision_3','palm_collision_4','palm_collision_6','palm_collision_7'}.issubset(names):
        raise ValueError('Expected palm collision pieces missing')
    if any(np.asarray(model.geom(name).bodyid).item()!=model.body('palm').id for name in names):
        raise ValueError('Compound palm contact contains different rigid bodies')
    return dict(body='palm',geom_names=sorted(names),pair=PAIR,verified_same_rigid_body=True,
        geometry_or_contact_forces_modified=False)
def metadata(raw):
    result=parent_metadata(raw);enabled=active(raw)
    if enabled:result['additional_initial_pairs']=result['additional_initial_pairs']+[PAIR]
    result['confirmed_initial_palm_body']=dict(active=enabled,pair=PAIR,maximum_initial_mm=MAX_INITIAL_MM,
        actual_thumb_palm_contact_user_confirmed=True,exact_mesh_face_registration_verified=False,
        confirmed_pose_ticks=REFERENCE.tolist(),route_selection_radius_ticks=32,
        evidence=EVIDENCE.name,evidence_sha256=hashlib.sha256(EVIDENCE.read_bytes()).hexdigest(),
        prefix=PREFIX.name,prefix_sha256=hashlib.sha256(PREFIX.read_bytes()).hexdigest(),
        initial_release_only=True,new_or_deeper_body_contacts_forbidden=True,
        static_monotonic_maximum_palm_depth_required=True,overall_initial4mm_limit_unchanged=True,
        native_face_contacts_aggregated_only_for_initial_release=True,
        raw_face_depths_recorded=True,contact_may_transfer_between_pieces_of_the_same_palm=True,
        collision_geometry_or_forces_disabled=False,hardware_health_limits_changed=False)
    return result
