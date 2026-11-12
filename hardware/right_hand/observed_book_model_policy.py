"""Scoped model correction for the measured passive book-supported OFF pose.

Only ID4's observed lower neighborhood is added, and generated recovery goals
may not move it farther down. A 0.1mm classification tolerance is confined to
the approximate palm-box/ring-tip pair. It does not remove contact forces.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
from hand_motor_mapping import TICK

ROOT=Path(__file__).resolve().parent
SOURCE='book_settled_off_readonly_20261024_174424_563949/request.json'
LIMITS='right_hand_rotation_20261023/native_motor_limits.json'
MAX_PASSIVE_START_EXCESS_RAD=float(np.deg2rad(10.))
CONTACT_PAIR='palm_collision_7|rf_tip'
CONTACT_TOLERANCE_MM=.1
SOFT_JOINT_RESIDUAL_RAD=TICK  # simulation state only; never goal/admission slack


def policy(raw=None):
    data=(ROOT/SOURCE).read_bytes();r=json.loads(data)
    s=r['off_state']
    if s['position'][3]!=1782 or any(any(s[k]) for k in ('torque','error','watchdog')):
        raise ValueError('Observed OFF envelope source changed')
    a=np.asarray(s['position'] if raw is None else raw)
    if a.shape!=(16,) or not np.isfinite(a).all():raise ValueError('Invalid observed model source')
    native=json.loads((ROOT/LIMITS).read_bytes())
    native_lower=float(native['lower_ticks'][3])
    model_lower=float(min(native_lower,int(a[3])-4))
    return dict(version='fresh_passive_ID4_reentry_ring_tip_box_tolerance_v2',
        observed_source=SOURCE,observed_source_sha256=hashlib.sha256(data).hexdigest(),
        original_limits_sha256=hashlib.sha256((ROOT/LIMITS).read_bytes()).hexdigest(),
        extended_motor_id=4,extended_lower_ticks=model_lower,
        start_ID4_ticks=int(a[3]),native_ID4_lower_ticks=native_lower,
        model_range_extended=model_lower<native_lower,
        maximum_passive_start_excess_rad=MAX_PASSIVE_START_EXCESS_RAD,
        lower_bound_from_each_fresh_read=True,
        only_reentry_toward_original_range=True,
        contact_pair=CONTACT_PAIR,contact_tolerance_mm=CONTACT_TOLERANCE_MM,
        soft_joint_residual_motor_id=4,soft_joint_residual_rad=SOFT_JOINT_RESIDUAL_RAD,
        contact_forces_preserved=True,physical_calibration_verified=False)


def bounds():
    policy()
    r=json.loads((ROOT/LIMITS).read_bytes())
    lo,hi=np.array(r['lower_ticks']),np.array(r['upper_ticks'])
    # Only an existing passive OFF pose is admitted here. Replay bounds use
    # that fresh source's start-4, never this entire10deg admission interval.
    lo[3]-=MAX_PASSIVE_START_EXCESS_RAD/TICK
    return lo,hi


def admission(values):
    a=np.asarray(values);lo,hi=bounds()
    if (a.shape!=(16,) or not np.isfinite(a).all() or np.any(a!=np.rint(a))
            or np.any(a<lo) or np.any(a>hi)):
        raise ValueError('Current pose outside measured recovery/model envelope; no writes')
    return a.astype(np.int64)


def healthy_off(s):
    a=admission(s['position'])
    for key in ('torque','error','watchdog','voltage','temperature'):
        v=np.asarray(s[key])
        if v.shape!=(16,) or not np.isfinite(v).all() or np.any(v!=np.rint(v)):
            raise ValueError('Invalid fresh OFF telemetry')
    if (any(np.any(s[k]) for k in ('torque','error','watchdog'))
            or np.any(np.asarray(s['voltage'])<38) or np.any(np.asarray(s['voltage'])>65)
            or np.any(np.asarray(s['temperature'])>45)):
        raise ValueError('Unhealthy OFF state; no writes')
    return a


def correct_model(model,raw=None):
    """Change only the observed ID4 lower joint/control bound in memory."""
    import mujoco
    before_joint=model.jnt_range.copy();before_ctrl=model.actuator_ctrlrange.copy()
    j=model.joint('th_cmc').id;a=model.actuator('th_cmc_act').id
    model_policy=policy(raw)
    model.jnt_range[j,0]=(model_policy['extended_lower_ticks']-2048)*TICK
    model.actuator_ctrlrange[a,0]=model.jnt_range[j,0]
    expected_joint=before_joint.copy();expected_ctrl=before_ctrl.copy()
    expected_joint[j,0]=model.jnt_range[j,0]
    expected_ctrl[a,0]=model.actuator_ctrlrange[a,0]
    if not np.array_equal(model.jnt_range,expected_joint) or not np.array_equal(model.actuator_ctrlrange,expected_ctrl):
        raise RuntimeError('Unexpected model range change')
    mujoco.mj_setConst(model,mujoco.MjData(model))
    return model_policy


def classify_self_contacts(pairs):
    return {p:v for p,v in pairs.items() if p!=CONTACT_PAIR or v>CONTACT_TOLERANCE_MM}
