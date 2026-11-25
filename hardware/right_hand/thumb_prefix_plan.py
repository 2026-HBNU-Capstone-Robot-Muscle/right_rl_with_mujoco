"""Bounded empty preparation observation, not SAC or a full thumb reset."""
import hashlib
import json
from pathlib import Path
import numpy as np
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from leap_pose_pair_readonly import validate_pair

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'pose_pair_readonly_20261021_183730_320810/result.json'
SOURCE_SHA='4dbad577311a454e31b57959c1650a9b18a6f96193e1641844d8c2bb21c5dc22'
ROOTS=np.array([7,11,15])
THUMB=np.arange(4)
FROZEN=np.array([i for i in range(16) if i not in (*ROOTS,*THUMB)])
TOLERANCE=32
MAX_THUMB_TRAVEL=96
ROOT_TARGET=2472
THUMB_TARGET=np.array([2601,3100,3074,2708])
STEPS=120

def source_position():
    content=SOURCE.read_bytes()
    if hashlib.sha256(content).hexdigest()!=SOURCE_SHA:raise ValueError('Reviewed paired source changed')
    r=json.loads(content)
    if r['status']!='photo_and_readings_saved' or not validate_pair(*r['snapshots'])['stationary_endpoints']:
        raise ValueError('Source photo/encoder pair invalid')
    for s in r['snapshots']:
        if s['joint_mapping']['motor_ids_in_sim_order']!=list(SIM_TO_MOTOR_IDS):raise ValueError('Source mapping changed')
    return np.array([m['position_ticks_signed'] for m in sorted(r['snapshots'][-1]['motors'],key=lambda m:m['id'])],dtype=np.int64)

def validate_start(raw):
    a=np.asarray(raw)
    if a.shape!=(16,) or not np.isfinite(a).all() or np.any(a!=np.rint(a)):raise ValueError('Invalid raw encoder array')
    reference=source_position()
    bad=np.flatnonzero(abs(a-reference)>TOLERANCE)
    if len(bad):
        raise ValueError('Current pose outside reviewed neighborhood; no writes: '+ '; '.join(f'ID{i+1} {int(a[i]-reference[i]):+d} ticks' for i in bad))
    if np.any(a[ROOTS]>=ROOT_TARGET):raise ValueError('Root target must be above current reviewed roots')
    return a.astype(np.int64)

def plan(raw):
    a=validate_start(raw)
    root=a.copy();root[ROOTS]=ROOT_TARGET
    end=root.copy()
    delta=THUMB_TARGET-a[:4]
    end[:4]=a[:4]+np.rint(delta*min(1.,MAX_THUMB_TRAVEL/max(abs(delta)))).astype(int)
    rows=[]
    for phase,b in [('stage_0_roots',root),('stage_1_thumb_prefix',end)]:
        for k in range(1,STEPS+1):
            q=np.rint(a+(b-a)*k/STEPS).astype(np.int64)
            rows.append((phase,q))
        a=b.copy()
    return rows
