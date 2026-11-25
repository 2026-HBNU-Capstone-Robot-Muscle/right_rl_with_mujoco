"""Fresh OFF recovery to the reverse reference's EMPTY cube placement pose."""
import hashlib
import json
import numpy as np
from book_grasp_trial_plan import (BookGraspPlan,ROOT,extra_hashes as parent_hashes,
    RING_RELEASE,THUMB_RELEASE)
from book_empty_preparation_plan import LIFT,PREPARE,ROOTS
from grasp_place_plan import endpoints
from grasp_recovery_plan import interpolate

APPROACH='stage_3_empty_reverse_start'
END_HOLD='stage_4_empty_start_hold'
TARGET=np.array([1915,2743,2622,3349,2126,2122,1819,2014,2160,2153,1818,2043,2411,2281,3081,2136])
FILES=('reverse_start_empty_plan_20261025.py','reverse_start_empty_gate_20261025.py',
    'screen_reverse_start_empty_20261025.py','leap_reverse_start_empty_20261025.py',
    'fingertip_cad_model_20261025.py')

def extra_hashes():
    return {**parent_hashes(),**{name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in FILES}}

class ReverseStartPlan(BookGraspPlan):
    PLAN_VERSION='reverse_quarter_turn_EMPTY_start_CAD_v1_20261025'
    TITLE='EMPTY REVERSE-90 START: ring release -> thumb release -> clearance -> new start -> OFF. NO CUBE.'
    COMPLETE_MESSAGE='New reverse-turn start reached EMPTY. OFF/settings restored. No physical cube turn was tested.'
    FOLDER_PREFIX='reverse_start_empty'
    STATUS='reverse_start_empty_complete'
    ON_TIMEOUT=100
    MAX_THUMB_TRAVEL=1700
    CAPTURE_PLACEMENT=False
    CAPTURE_BEFORE_PHASE=None
    EMPTY_RELEASE_ONLY=False
    ROTATION_INCLUDED=False
    proof_hashes=staticmethod(extra_hashes)

    def plan(self,raw):
        a=self.validate_start(raw);ring=a.copy();ring[14]-=80
        tip=ring.copy();tip[0]-=64;roots=tip.copy();roots[ROOTS]=np.maximum(roots[ROOTS],2472)
        opened,_=endpoints();rows=[]
        for start,end,phase in ((a,ring,RING_RELEASE),(ring,tip,THUMB_RELEASE),(tip,roots,LIFT),
            (roots,opened,PREPARE),(opened,TARGET,APPROACH)):
            interpolate(rows,start,end,phase)
        rows.append((END_HOLD,TARGET.copy()))
        if (len(rows)+20)*.06+2>self.ON_TIMEOUT:raise ValueError('EMPTY entry exceeds its bounded delayed-cycle budget')
        if max(abs(TARGET[:4]-a[:4]))>self.MAX_THUMB_TRAVEL:raise ValueError('Thumb travel exceeds the dedicated entry bound')
        archive=json.loads((ROOT/'sequence_flip_poses.json').read_bytes())
        if not np.array_equal(TARGET,archive['poses'][2]['raw_ticks']):raise ValueError('Original teacher pose changed')
        from observed_book_model_policy import bounds
        low,high=bounds();values=np.array([goal for _,goal in rows])
        if np.any(values<low) or np.any(values>high):raise ValueError('New EMPTY goals outside observed/native admission')
        native_lower=json.loads((ROOT/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())['lower_ticks'][3]
        previous=int(a[3])
        for _,goal in rows:
            if goal[3]<native_lower and goal[3]<previous:raise ValueError('Entry would deepen the passive thumb extension')
            previous=int(goal[3])
        return rows

    def frozen_for_phase(self,phase):
        if phase==END_HOLD:return np.arange(16)
        if phase==APPROACH:return np.array([],dtype=int)
        return super().frozen_for_phase(phase)

    def progress_for_phase(self,phase):
        if phase in (APPROACH,PREPARE):return np.arange(16)
        if phase==END_HOLD:return np.array([],dtype=int)
        return super().progress_for_phase(phase)
