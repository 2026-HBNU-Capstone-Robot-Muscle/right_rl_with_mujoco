"""Explicit joint/ID conversion for offline review and read-only display.

All four tip-to-root orders were explicitly supplied by the user on 2026-10-18.
This module alone does not validate signs, geometry or a robot trajectory.
No serial imports, control calls, offsets fitted to photographs, or clipping.
"""
import numpy as np

MAPPING_VERSION='all_finger_orders_user_confirmed_20261018_v2'
SIM_JOINT_NAMES=('if_mcp','if_rot','if_pip','if_dip',
                 'mf_mcp','mf_rot','mf_pip','mf_dip',
                 'rf_mcp','rf_rot','rf_pip','rf_dip',
                 'th_cmc','th_axl','th_mcp','th_ipl')
SIM_TO_MOTOR_IDS=(8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1)
LEGACY_SIM_TO_MOTOR_IDS=(5,6,7,8,9,10,11,12,13,14,15,16,1,2,3,4)
SIM2ARR=np.array(SIM_TO_MOTOR_IDS)-1
TICK=2*np.pi/4096
assert sorted(SIM_TO_MOTOR_IDS)==list(range(1,17))

def _values(values):
    a=np.asarray(values,dtype=float)
    if a.ndim<1 or a.shape[-1]!=16 or not np.isfinite(a).all():
        raise ValueError('Expected finite (...,16) joint values')
    return a

def hw_to_sim(hw_angles):
    return _values(hw_angles)[...,SIM2ARR]-np.pi

def ticks_to_sim(ticks):
    return hw_to_sim(_values(ticks)*TICK)

def sim_to_hw(sim_angles):
    a=_values(sim_angles);result=np.empty_like(a)
    result[...,SIM2ARR]=a+np.pi
    return result

def sim_to_ticks(sim_angles):
    return np.rint(sim_to_hw(sim_angles)/TICK).astype(np.int64)

def metadata():
    return dict(version=MAPPING_VERSION,sim_joint_names=list(SIM_JOINT_NAMES),
                motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
                middle_tip_to_root_ids=[9,11,10,12],
                tip_to_root_ids={'thumb':[1,2,3,4],'index':[5,7,6,8],
                                 'middle':[9,11,10,12],'ring':[13,15,14,16]},
                middle_order_basis='User explicit physical order confirmation, 2026-10-18; saved tip flexion ID9 +9.492 degrees.',
                other_fingers_basis='User explicitly answered the tip-to-root ID question: thumb 1,2,3,4; index 5,7,6,8; ring 13,15,14,16.',
                geometric_calibration_verified=False,trajectory_validation_complete=False)

def reject_legacy_sac_execution():
    """Historical exports/preparation use unverified ascending IDs as joint order."""
    raise RuntimeError('Old SAC path/start uses obsolete middle mapping (ID9 root / ID12 tip). '
                       'All confirmed root-to-tip groups: index 8,6,7,5; middle 12,10,11,9; '
                       'ring 16,14,15,13; thumb 4,3,2,1. '
                       'Offline remapping and preparation/collision validation are pending; no motor port opened.')
