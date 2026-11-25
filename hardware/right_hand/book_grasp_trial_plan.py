"""Book-aware preparation and one previously reached slow grasp approach.

The native screen certifies the hand route only. Cube contact and grasp remain
experimental and require explicit PLACE/REMOVE checkpoints and real observation.
"""
import hashlib
import json
from functools import lru_cache
import numpy as np
from book_empty_preparation_plan import BookPreparationPlan, ROOT, ROOTS, LIFT,PREPARE
from grasp_place_plan import endpoints, LOADED_PHASE
from grasp_recovery_plan import interpolate
from observed_book_model_policy import admission as observed_admission,healthy_off as observed_healthy_off
from book_passive_support_reentry_policy import (INITIAL_SUPPORT_DEPTH_MM as MAX_INITIAL_SUPPORT_MM,
    ROOT_ESCAPE_GOALS as SUPPORT_ROOT_ESCAPE_GOALS,ROOT_ESCAPE_SECONDS,
    INITIAL_MODEL_TRACKING_RAD as INITIAL_CONTACT_TRACKING_RAD)

EXTRA_FILES=('book_grasp_trial_plan.py','book_grasp_trial_gate.py',
    'screen_book_grasp_trial.py','leap_book_cube_grasp_trial.py',
    'grasp_place_capture.py','prepared_placement_capture.py',
    'book_thumb_release_guard.py',
    'camera_property_confirmation.py','camera_exposure_sweep.py',
    'book_grasp_reached_prefix_20261024.json',
    'book_cube_grasp_trial_20261024_172726_286561/result.json',
    'book_post_off_contact_observation_20261024.json',
    'observed_book_model_policy.py',
    'book_settled_off_readonly_20261024_174424_563949/request.json',
    'book_passive_support_reentry_policy.py',
    'book_settled_off_readonly_20261024_212026_419273/request.json')
THUMB_RELEASE='stage_0_thumb_tip_release'
RING_RELEASE='stage_0_ring_pip_release'
RING_RELEASE_TICKS=80
THUMB_RELEASE_TICKS=64
# The observed OFF neighborhood touches these three neighboring fixed palm
# primitives. Only the existing pairs may separate during the 40-goal release;
# they remain forbidden after it, apart from the scoped 0.1mm pair tolerance.
RING_SELF_PAIRS=('palm_collision_4|rf_tip','palm_collision_6|rf_tip','palm_collision_7|rf_tip')
SELF_PAIRS=('palm_collision_4|th_tip','palm_collision_8|th_tip')+RING_SELF_PAIRS
# Immediate post-OFF and later stable READ-only snapshots differ as the thumb
# settles. Their complete +/-4 neighborhoods reach 2.68 and 3.82mm in this
# unregistered collision model. Bound the reviewed initial contact at 4mm,
# while requiring every pair to decrease and clear within only 32 goals.
# This geometric envelope is not a physical penetration/force measurement.
MAX_INITIAL_THUMB_SELF_MM=4.
THUMB_ESCAPE_GOALS=32


@lru_cache(maxsize=1)
def reached_prefix():
    """An exact strict prefix of goals observed in the completed recording."""
    meta=json.loads((ROOT/'book_grasp_reached_prefix_20261024.json').read_bytes())
    source=ROOT/meta['source'];data=source.read_bytes()
    if hashlib.sha256(data).hexdigest()!=meta['source_sha256']:raise ValueError('Observed grasp trial changed')
    r=json.loads(data)
    if (not r.get('all_torque_off_verified') or not r.get('settings_restored')
        or not r.get('cube_placement_confirmed') or r.get('policy_used')
        or r.get('error')!='ID12 no progress over 20 fresh samples with >=64 tick error; no target advance'):
        raise ValueError('Unverified grasp prefix source')
    opened,closed=endpoints();full=[];interpolate(full,opened,closed,LOADED_PHASE)
    digest=hashlib.sha256(json.dumps([q.tolist() for _,q in full],separators=(',',':')).encode()).hexdigest()
    count=meta['loaded_target_count'];observed=[s for s in r['samples'] if s['phase']==LOADED_PHASE][279]
    if (count!=279 or len(full)!=322 or digest!=meta['old_path_target_sha256']
        or full[count-1][1].tolist()!=meta['shortened_goal'] or observed['goal']!=meta['shortened_goal']
        or max(observed['tracking_rad'])>.09 or max(abs(v) for v in observed['current_ma'])>150):
        raise ValueError('Shortened grasp is not the bounded observed command prefix')
    return tuple(tuple(q.tolist()) for _,q in full[:count])


def reached_grasp_goal():return np.array(reached_prefix()[-1],dtype=int)


def extra_hashes():
    return {name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in EXTRA_FILES}


class BookGraspPlan(BookPreparationPlan):
    PLAN_VERSION='book20_fresh_passive_reentry_frames_only_grasp_v7'
    TITLE='BOOK GRASP: EMPTY ring release -> thumb release -> preparation -> PLACE -> reached grasp prefix -> REMOVE -> OFF.'
    COMPLETE_MESSAGE='Grasp approach completed; cube removal confirmed; OFF/settings restored. Rotation was not tested.'
    FOLDER_PREFIX='book_cube_grasp_trial'
    STATUS='book_cube_grasp_trial_complete'
    ON_TIMEOUT=75
    MODEL_LIMITS_DESCRIPTION='measured OFF recovery range and initial-contact separation'
    CAPTURE_PLACEMENT=True
    CAPTURE_BEFORE_PHASE=LOADED_PHASE
    CHECKPOINT_SETTLE_SAMPLES=20
    CHECKPOINT_MAX_ERROR_TICKS=64
    PLACEMENT_PAUSE_LIMIT=100
    validate_admission=staticmethod(observed_admission)
    validate_off_state=staticmethod(observed_healthy_off)

    def plan(self,raw):
        a=self.validate_start(raw);ring=a.copy();ring[14]-=RING_RELEASE_TICKS
        self.validate_admission(ring)
        rows=[];interpolate(rows,a,ring,RING_RELEASE)
        tip=ring.copy();tip[0]-=THUMB_RELEASE_TICKS
        self.validate_admission(tip)
        interpolate(rows,ring,tip,THUMB_RELEASE)
        root=tip.copy();root[ROOTS]=np.maximum(root[ROOTS],2472)
        interpolate(rows,tip,root,LIFT)
        opened,closed=endpoints()
        nt=max(20,int(np.ceil(max(abs(opened[:4]-root[:4]))/2)))
        nf=max(20,int(np.ceil(max(abs(opened[4:]-root[4:]))/2)))
        for k in range(1,max(nt,nf)+1):
            q=root.copy()
            q[:4]=np.rint(root[:4]+(opened[:4]-root[:4])*min(1.,k/nt)).astype(int)
            q[4:]=np.rint(root[4:]+(opened[4:]-root[4:])*min(1.,k/nf)).astype(int)
            rows.append((PREPARE,q))
        if (len(rows)+20)*.06+2>55:
            raise ValueError('Empty preparation exceeds its original 55s budget')
        if not np.array_equal(rows[-1][1],opened):
            raise ValueError('Placement target differs from completed empty preparation')
        rows.extend((LOADED_PHASE,np.array(q,dtype=int)) for q in reached_prefix())
        # An observed passive extension is admitted for recovery, never as a
        # request to drive farther beyond the original thumb-base lower bound.
        original_lower=json.loads((ROOT/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())['lower_ticks'][3]
        if a[3]<original_lower:
            previous=int(a[3])
            for _,q in rows:
                if q[3]<original_lower and q[3]<previous:
                    raise ValueError('Recovery would deepen the passive thumb extension')
                previous=int(q[3])
        # Include settling and final hold. Operator PLACE/REMOVE waiting is
        # separately bounded and health checked, never charged as motion time.
        if (len(rows)+40)*.06+2>self.ON_TIMEOUT:
            raise ValueError('Combined preparation/grasp exceeds the delayed-cycle 75s budget')
        return rows

    def frozen_for_phase(self,phase):
        if phase==RING_RELEASE:return np.array([i for i in range(16) if i!=14])
        if phase==THUMB_RELEASE:return np.arange(1,16)
        if phase==LOADED_PHASE:return np.arange(12,16)
        if phase in ('placement_checkpoint','stage_2_fixed_hold'):return np.arange(16)
        return super().frozen_for_phase(phase)

    def progress_for_phase(self,phase):
        if phase==RING_RELEASE:return np.array([14])
        if phase==THUMB_RELEASE:return np.array([0])
        if phase==LOADED_PHASE:return np.arange(12)
        if phase=='placement_checkpoint':return np.array([],dtype=int)
        return super().progress_for_phase(phase)

