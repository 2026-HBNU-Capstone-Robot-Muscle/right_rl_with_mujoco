"""Restore the reached reverse start and photograph Cube placement, no turn."""
import hashlib,json
import numpy as np
from reverse_start_empty_plan_20261025 import ReverseStartPlan,ROOT,TARGET,APPROACH,END_HOLD,RING_RELEASE,THUMB_RELEASE,extra_hashes as previous_hashes
from book_empty_preparation_plan import ROOTS,LIFT
from grasp_recovery_plan import interpolate

PHOTO_HOLD='stage_5_fixed_cube_photo_hold'
BASE_RELEASE='stage_0_thumb_base_backoff'
CASING_PAIR='palm_collision_8|th_ds_collision_1'
CASING_INITIAL_LIMIT_MM=1.
INITIAL_SUPPORT_SCOPE='OFF_4_tick_and_fixed_enable_hold_support_envelope_v1'
SOURCE='reverse_start_empty_20261025_043035_076434/result.json'
SOURCE_SHA='975b3089a3cb47e776d498852c7aee789d8b33a0a9a3e61ca5f127ad5aa2cddc'
FILES=('reverse_place_capture_plan_20261025.py','reverse_place_capture_protocol_20261025.py',
    'reverse_place_capture_gate_20261025.py','screen_reverse_place_capture_20261025.py',
    'leap_reverse_place_capture_20261025.py',SOURCE)

def extra_hashes():return {**previous_hashes(),**{name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in FILES}}

class ReversePlacePlan(ReverseStartPlan):
    PLAN_VERSION='reverse_start_fixed_placement_capture_v3_early_roots_20261025'
    TITLE='NEW START PLACEMENT PHOTO: EMPTY restore -> PLACE/photo -> REMOVE -> OFF. NO TURN.'
    COMPLETE_MESSAGE='New start placement saved; removal confirmed; OFF/settings restored. No rotation was sent.'
    FOLDER_PREFIX='reverse_start_placement'
    STATUS='reverse_start_placement_complete'
    CAPTURE_PLACEMENT=True
    CAPTURE_BEFORE_PHASE=PHOTO_HOLD
    ROTATION_INCLUDED=False
    proof_hashes=staticmethod(extra_hashes)

    def plan(self,raw):
        a=self.validate_start(raw);blob=(ROOT/SOURCE).read_bytes();record=json.loads(blob)
        if (hashlib.sha256(blob).hexdigest()!=SOURCE_SHA or record['status']!='reverse_start_empty_complete'
            or not record['all_torque_off_verified'] or not record['settings_restored']
            or not np.array_equal(record['final_command'],TARGET)):
            raise ValueError('Completed EMPTY start evidence changed')
        if np.max(abs(a-TARGET))>192:
            raise ValueError('Current posture left the reached start neighborhood; no writes; review the saved fresh readings')
        # Restore the thumb-base command from the completed ON run first.
        # Clear the existing OFF casing/palm corner while the roots start up.
        base=a.copy();base[3]=min(int(a[3]),int(TARGET[3]))
        # The roots can sag farther after OFF. Begin their upward separation
        # during the short base release, preserving the 2-tick/50ms speed cap.
        # Waiting for the base first spent too much of the support deadline.
        base_steps=max(20,int(np.ceil(abs(int(base[3])-int(a[3]))/2)))
        base[ROOTS]+=np.minimum(2*base_steps,np.maximum(0,TARGET[ROOTS]-a[ROOTS]))
        rows=[];interpolate(rows,a,base,BASE_RELEASE)
        if sum(p==BASE_RELEASE for p,_ in rows)>32:raise ValueError('Thumb-base separation exceeds its short bounded release')
        roots=base.copy();roots[ROOTS]=TARGET[ROOTS]
        interpolate(rows,base,roots,LIFT);interpolate(rows,roots,TARGET,APPROACH)
        rows.append((END_HOLD,TARGET.copy()))
        rows.extend((PHOTO_HOLD,TARGET.copy()) for _ in range(20))
        if (len(rows)+40)*.06+2>self.ON_TIMEOUT:raise ValueError('Placement preparation exceeds bounded motion budget')
        return rows

    def frozen_for_phase(self,phase):
        if phase==BASE_RELEASE:return np.array([i for i in range(16) if i not in (3,7,11,15)])
        if phase in (PHOTO_HOLD,'placement_checkpoint'):return np.arange(16)
        return super().frozen_for_phase(phase)

    def progress_for_phase(self,phase):
        if phase==BASE_RELEASE:return np.array([3,7,11,15])
        if phase in (PHOTO_HOLD,'placement_checkpoint'):return np.array([],dtype=int)
        return super().progress_for_phase(phase)
