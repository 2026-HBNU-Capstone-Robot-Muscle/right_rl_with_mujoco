"""Measured passive root admission for this empty final-pose command only."""
import json
from pathlib import Path
import numpy as np
from observed_book_model_policy import bounds as previous_bounds, healthy_off as previous_health
from loaded_quarter_reentry_20261025 import limits
from loaded_quarter_model_policy_20261025 import correct_model as previous_model
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'final_grip_pose_20261116_152024_215914/result.json'
ROOT_INDICES=np.asarray([6,7,11,15])
# Fresh OFF-only sample cube_grasp_hold_20261116_200443_746252:
# ID7=1706,ID8=1816,ID12=1823,ID16=1820, plus4tick review radius.
PASSIVE_LOWER=np.asarray([1702,1812,1819,1816])

def admission(values):
    a=np.asarray(values);lo,hi=previous_bounds();lo[ROOT_INDICES]=PASSIVE_LOWER
    if (a.shape!=(16,) or not np.isfinite(a).all() or np.any(a!=np.rint(a))
            or np.any(a<lo) or np.any(a>hi)):
        bad=[] if a.shape!=(16,) else [f'ID{i+1}={a[i]} allowed [{lo[i]:.1f},{hi[i]:.1f}]' for i in range(16) if not lo[i]<=a[i]<=hi[i]]
        raise ValueError('Final-pose start outside measured admission: '+'; '.join(bad)+'; no writes')
    return a.astype(int)

def healthy_off(state):
    a=admission(state['position']);copy=dict(state)
    # Reuse every old health check, without letting its old range check reject
    # the three separately admitted passive root readings.
    native,_=limits();probe=a.copy();probe[ROOT_INDICES]=np.maximum(probe[ROOT_INDICES],np.ceil(native[ROOT_INDICES]).astype(int))
    copy['position']=probe;previous_health(copy)
    return a

def validate_path(raw,rows,entry):
    a=admission(raw);values=np.vstack([a]+[q for _,q in rows]);lo,hi=limits()
    if not np.isfinite(values).all() or np.any(values!=np.rint(values)) or np.max(abs(np.diff(values,axis=0)))>2:
        raise ValueError('Final-pose goal shape/integer/2tick contract rejected')
    passive={3,6,7,11,15}
    for j in range(16):
        if np.any(values[:,j]>hi[j]):raise ValueError(f'ID{j+1} exceeds native upper goal')
        if j not in passive and np.any(values[:,j]<lo[j]):raise ValueError(f'ID{j+1} exceeds unchanged native lower goal')
        if j in passive:
            previous=int(a[j]);restored=previous>=lo[j]
            for phase,q in rows:
                current=int(q[j])
                if not restored:
                    if phase!=entry or current<previous:
                        raise ValueError(f'ID{j+1} passive reentry must be nondecreasing in initial phase only')
                    restored=current>=lo[j]
                elif current<lo[j]:raise ValueError(f'ID{j+1} left native range after reentry')
                previous=current
            if not restored:raise ValueError(f'ID{j+1} did not return to native goal range')
    return values

def correct_model(model,raw=None):
    import mujoco
    r=previous_model(model,raw);a=admission(raw);lo,_=limits();extended=[]
    for physical in ROOT_INDICES:
        sim=int(np.flatnonzero(np.asarray(SIM_TO_MOTOR_IDS)-1==physical)[0])
        # First16 qpos and actuators follow SIM_TO_MOTOR_IDS in this model.
        joint=int(np.flatnonzero(model.jnt_qposadr==sim)[0]);act=sim
        lower=min(float(lo[physical]),float(a[physical]-4))*TICK-2048*TICK
        model.jnt_range[joint,0]=lower;model.actuator_ctrlrange[act,0]=lower
        extended.append(dict(id=int(physical+1),passive_model_lower_ticks=float(lower/TICK+2048)))
    mujoco.mj_setConst(model,mujoco.MjData(model))
    r['final_pose_passive_roots']=extended
    return r
