"""EMPTY roots-first preparation measured anew for the 20mm book arrangement.

This is a separate experimental contact-separation plan. It does not certify
free space, register the physical hand, or alter any previous motion gate.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
from thumb_live_start_plan import CurrentPlan
from grasp_place_plan import endpoints
from grasp_recovery_plan import interpolate
from book_support_contact_scope import allowed_initial_pairs

ROOT=Path(__file__).resolve().parent
REFERENCE=ROOT/'book_pose_readonly_20261024_054007_999816/result.json'
LIMITS=ROOT/'right_hand_rotation_20261023/native_motor_limits.json'
ROOTS=np.array([7,11,15])
LIFT='stage_0_book_clearance'
TIP_CLEARANCE='stage_1_thumb_tip_clearance'
PREPARE='stage_2_empty_preparation'
FRONTS=(-15.,-10.,-5.)  # photo sensitivity cases, not measured confidence bounds
MAX_INITIAL_SUPPORT_MM=4.  # bounds this dedicated escape only, never free-space acceptance
SUPPORT_ESCAPE_STATIC_GOALS=40
SUPPORT_ESCAPE_DYNAMIC_SECONDS=3.5
ALLOWED_SUPPORT_PAIRS=allowed_initial_pairs()
SOURCE_FILES=(
    'book_empty_preparation_plan.py','book_empty_preparation_gate.py',
    'book_support_contact_scope.py','book_support_escape_scope_20261024.json',
    'screen_book_empty_preparation.py','book_support_model_20261024.py',
    'current_book_support_measurements_20261024.json','sac_wood_glass_model.py',
    'wood_glass_support_measurements.json','hand_motor_mapping.py',
    'initial_contact_escape.py','thumb_live_start_plan.py','thumb_live_start_gate.py',
    'grasp_place_plan.py','grasp_recovery_plan.py','grasp_approach_plan.py',
    'leap_book_prepare_empty.py','leap_thumb_prepare_current_empty.py',
    'leap_thumb_prefix_empty.py','sequence_flip_checked.py','held_joint_guard.py',
    'goal_readback_confirmation.py','marker_trial_camera.py','fixed_exposure_camera.py',
    'empty_trial_camera.py','empty_start_marker_check.py','camera_frame_signal.py','camera_stream.py',
    'sac_cube_model.py','sac_stage0_reset_audit_result.json',
    'grasp_approach_current_empty_20261022_020751_741219/result.json',
    'right_hand_rotation_20261023/native_motor_limits.json',
    'book_pose_readonly_20261024_054007_999816/result.json',
)


def reference_ticks():
    r=json.loads(REFERENCE.read_bytes())
    if r['status']!='photo_and_readings_saved' or r['pair_check']['maximum_drift_ticks']!=0:
        raise ValueError('Photographed OFF reference is not stationary')
    return np.array([next(v['position_ticks_signed'] for v in r['snapshots'][0]['motors']
                         if v['id']==i) for i in range(1,17)],dtype=np.int64)


def admission(raw):
    a=np.asarray(raw)
    if (a.shape!=(16,) or not np.isfinite(a).all() or np.any(a!=np.rint(a))
            or np.any(a<0) or np.any(a>4095)):
        raise ValueError('Invalid raw posture; no wrapping or correction')
    # Admission to simulation uses current raw/native limits, not proximity to
    # yesterday's photo. Actual writes still require stable OFF measurements,
    # a bounded path/time budget and all exact-start neighborhood/dynamics
    # screens. A different posture is never approved by this function alone.
    limits=json.loads(LIMITS.read_bytes());lo=np.array(limits['lower_ticks']);hi=np.array(limits['upper_ticks'])
    if np.any(a<lo) or np.any(a>hi):raise ValueError('Current pose outside native limits; no writes')
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


def offsets():
    rows=[np.zeros(16,dtype=int),np.full(16,-4),np.full(16,4)]
    for i in range(16):
        for sign in (-1,1):
            a=np.zeros(16,dtype=int);a[i]=4*sign;rows.append(a)
    rows.extend(np.random.default_rng(20261021).integers(-4,5,(32,16)))
    return rows


def code_hashes():
    return {name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in SOURCE_FILES}


class BookPreparationPlan(CurrentPlan):
    PLAN_VERSION='book20_native_fresh_structural_initial_contact_preparation_v4'
    TITLE='EMPTY BOOK-SUPPORT PREPARATION: roots -> thumb-tip clearance -> preparation -> 1s hold -> OFF.'
    COMPLETE_MESSAGE='Empty preparation complete; OFF/settings restored. No cube or rotation was tested.'
    FOLDER_PREFIX='book_prepare_empty'
    STATUS='book_empty_preparation_complete'
    ON_TIMEOUT=55
    MAX_THUMB_TRAVEL=1100
    MAX_GOAL_STEP=2
    SAMPLE_PERIOD=.05
    HELD_JOINT_GUARD=True
    CONFIRM_PREVIOUS_GOAL_READBACK=True
    validate_admission=staticmethod(admission)
    validate_off_state=staticmethod(healthy_off)

    def plan(self,raw):
        a=self.validate_start(raw);root=a.copy()
        root[ROOTS]=np.maximum(root[ROOTS],2472)
        opened,_=endpoints()
        rows=[];interpolate(rows,a,root,LIFT)
        # The direct interpolation failed the actual +/-4 screen at the
        # thumb tip/index base. A 64-tick tip waypoint clears that crossing.
        # No self-contact tolerance is increased to accept the failed path.
        detour=root.copy();detour[0]-=64
        interpolate(rows,root,detour,TIP_CLEARANCE)
        nt=max(20,int(np.ceil(max(abs(opened[:4]-detour[:4]))/2)))
        nf=max(20,int(np.ceil(max(abs(opened[4:]-detour[4:]))/2)))
        for k in range(1,max(nt,nf)+1):
            q=detour.copy()
            q[:4]=np.rint(detour[:4]+(opened[:4]-detour[:4])*min(1.,k/nt)).astype(int)
            q[4:]=np.rint(detour[4:]+(opened[4:]-detour[4:])*min(1.,k/nf)).astype(int)
            rows.append((PREPARE,q))
        if (len(rows)+20)*.06+2>self.ON_TIMEOUT:
            raise ValueError('Preparation exceeds delayed-cycle 55s budget')
        return rows

    def frozen_for_phase(self,phase):
        if phase==LIFT:return np.array([i for i in range(16) if i not in ROOTS])
        if phase==TIP_CLEARANCE:return np.arange(1,16)
        return np.arange(16) if phase=='stage_2_fixed_hold' else np.array([],dtype=int)

    def progress_for_phase(self,phase):
        if phase==LIFT:return ROOTS
        if phase==TIP_CLEARANCE:return np.array([0])
        return np.arange(16) if phase==PREPARE else np.array([],dtype=int)
