"""Pure current-pose plan for the remainder of EMPTY thumb preparation.

Only the measured family of open non-thumb postures is supported. Each actual
start must receive a separate offline geometry/dynamics screen before writes.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
from hand_motor_mapping import SIM_TO_MOTOR_IDS

ROOT=Path(__file__).resolve().parent
REFERENCE=ROOT/'thumb_continuation_empty_20261021_194300_889550/result.json'
REFERENCE_SHA='7b46fd6511990613376c4f2d70c490b73c644b2d86ede5ab9cd7f05be45891cf'
ROOTS=np.array([7,11,15]);THUMB=np.arange(4)
FROZEN=np.array([i for i in range(16) if i not in (*ROOTS,*THUMB)])
THUMB_TARGET=np.array([2601,3100,3074,2708])
ROOT_TARGET=2472
ROOT_STEPS,THUMB_STEPS=128,480
MAX_GOAL_STEP=2
MAX_THUMB_TRAVEL=950
PLAN_VERSION='empty_thumb_remainder_fresh_start_v1'


def reviewed_reference():
    data=REFERENCE.read_bytes()
    if hashlib.sha256(data).hexdigest()!=REFERENCE_SHA:raise ValueError('Completed thumb source changed')
    r=json.loads(data)
    if (r['status']!='empty_thumb_continuation_complete' or not r['settings_restored']
            or not r['all_torque_off_verified']
            or r['joint_mapping']['motor_ids_in_sim_order']!=list(SIM_TO_MOTOR_IDS)):
        raise ValueError('Completed thumb reference invalid')
    return np.array(r['final_on_state']['position'],dtype=np.int64)


def validate_current_family(raw):
    a=np.asarray(raw)
    if a.shape!=(16,) or not np.isfinite(a).all() or np.any(a!=np.rint(a)):
        raise ValueError('Invalid current raw angles')
    reference=reviewed_reference()
    if np.any(abs(a[FROZEN]-reference[FROZEN])>32):
        raise ValueError('Non-thumb posture changed; new geometry review required, no writes')
    # Admission only: thumb must be within the bounded remaining travel.
    # Actual native joint/control limits and collisions are screened in WSL.
    # In particular, OFF sag is not restricted to an assumed last ON angle.
    if (np.any(a<0) or np.any(a>4095)
            or np.any(abs(a[:4]-THUMB_TARGET)>MAX_THUMB_TRAVEL)
            or np.any(a[ROOTS]<2230) or np.any(a[ROOTS]>2500)):
        raise ValueError('Current pose outside the reviewed thumb-preparation family; no writes')
    return a.astype(np.int64)


def validate_off_state(s):
    a=validate_current_family(s['position'])
    if (any(np.any(s[k]) for k in ('torque','error','watchdog'))
            or np.any(np.asarray(s['voltage'])<38) or np.any(np.asarray(s['voltage'])>65)
            or np.any(np.asarray(s['temperature'])>45)):
        raise ValueError('Fresh OFF state not healthy; no writes')
    return a


class CurrentPlan:
    ROOTS=ROOTS;THUMB=THUMB;FROZEN=FROZEN
    MAX_THUMB_TRAVEL=MAX_THUMB_TRAVEL;MAX_GOAL_STEP=MAX_GOAL_STEP
    PLAN_VERSION=PLAN_VERSION
    ON_TIMEOUT=55
    SAMPLE_PERIOD=.05
    STATUS='empty_thumb_remainder_complete'
    def __init__(self,request_path):
        self.SOURCE=Path(request_path)
        data=self.SOURCE.read_bytes();self.SOURCE_SHA=hashlib.sha256(data).hexdigest()
        request=json.loads(data)
        if (request['plan_version']!=self.PLAN_VERSION
                or request['motor_ids_in_sim_order']!=list(SIM_TO_MOTOR_IDS)):
            raise ValueError('Request mapping/version changed')
        validator=getattr(self,'validate_off_state',validate_off_state)
        self.center=validator(request['off_state'])

    def source_position(self):
        if hashlib.sha256(self.SOURCE.read_bytes()).hexdigest()!=self.SOURCE_SHA:
            raise ValueError('Fresh request changed after screening')
        return self.center.copy()

    def validate_start(self,raw):
        validator=getattr(self,'validate_admission',validate_current_family)
        a=validator(raw)
        if np.any(abs(a-self.source_position())>4):
            raise ValueError('Pose moved beyond the screened four-tick neighborhood; no writes')
        return a

    def plan(self,raw):
        a=self.validate_start(raw);root=a.copy()
        root[ROOTS]=np.maximum(a[ROOTS],ROOT_TARGET)
        end=root.copy();end[:4]=THUMB_TARGET
        if max(abs(end[:4]-a[:4]))>MAX_THUMB_TRAVEL:
            raise ValueError('Remaining thumb travel exceeds bound')
        rows=[]
        for phase,b,count in [('stage_0_roots',root,ROOT_STEPS),
                              ('stage_1_thumb_prefix',end,THUMB_STEPS)]:
            for k in range(1,count+1):
                rows.append((phase,np.rint(a+(b-a)*k/count).astype(np.int64)))
            a=b.copy()
        return rows

    def plan_digest(self,raw):
        payload=[(phase,q.tolist()) for phase,q in self.plan(raw)]
        return hashlib.sha256(json.dumps(payload,separators=(',',':')).encode()).hexdigest()
