"""Explicit sub-resolution allowance for compliant, held-thumb/book contact.

No geometry, force, control target, or hardware threshold is changed. The
allowance cannot admit any commanded geometric penetration: every static
case must retain zero contact for this pair. It applies only to the finite-PD
initial hold and EMPTY root lift, and only to the reviewed open-middle family.
"""
import numpy as np
PAIR='support_book|th_ds_collision_1'
MAX_MM=.02
PHASES=('initial_hold','stage_1_roots_clear')
POLICY=dict(version='held_thumb_book_compliance_numeric_v1_20261027',pair=PAIR,
    maximum_dynamic_depth_mm=MAX_MM,maximum_commanded_static_depth_mm=1e-6,
    phase_names=list(PHASES),motor_ids_unchanged=True,contact_forces_preserved=True,
    geometry_unchanged=True,physical_calibration_verified=False)

def enabled(raw):
    a=np.asarray(raw)
    return bool(a.shape==(16,) and a[1]<2304 and a[8]<2304 and a[3]>=1872)

def allowance(raw,phase,pair):
    return MAX_MM if enabled(raw) and phase in PHASES and pair==PAIR else 0.

def review(plan,empty):
    active=enabled(plan.source_position());static=empty.get('static_cases',[]);dynamic=empty.get('dynamic_cases',[])
    if active:
        raw=plan.source_position()
        if any(not np.array_equal(q[:4],raw[:4]) for phase,q in plan.plan(raw) if phase=='stage_1_roots_clear'):
            raise RuntimeError('Held-thumb allowance requires fixed thumb goals throughout root recovery')
    for row in static:
        value=row.get('commanded_thumb_book_mm')
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not np.isfinite(value) or value<0 or value>1e-6:
            raise RuntimeError('Held-thumb allowance cannot admit commanded book penetration')
    for row in dynamic:
        value=row.get('raw_held_thumb_book_mm')
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not np.isfinite(value) or value<0 or value>(MAX_MM if active else 1e-6):
            raise RuntimeError('Held-thumb book contact exceeded its bounded dynamic allowance')
    return dict(enabled=active,policy=dict(POLICY),maximum_raw_held_depth_mm=max((r['raw_held_thumb_book_mm'] for r in dynamic),default=0.))
