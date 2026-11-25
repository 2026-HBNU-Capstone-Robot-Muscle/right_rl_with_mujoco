"""Next EMPTY finger preparation after the measured thumb target was reached."""
import numpy as np
from thumb_live_start_plan import CurrentPlan,ROOTS,FROZEN,THUMB_TARGET,ROOT_TARGET

FULL_TARGET=np.array([2601,3100,3074,2708,2857,2191,2081,2472,
                      2857,2042,2185,2472,3238,2112,2625,3047])
FINGER_STEPS=512


class PreparationPlan(CurrentPlan):
    PLAN_VERSION='empty_hand_fingers_after_thumb_v1'
    EXTRA_PLANNER_FILES=('hand_preparation_plan.py',)
    STATUS='empty_hand_preparation_complete'
    SAMPLE_PERIOD=.05
    ON_TIMEOUT=55
    TITLE='EMPTY HAND PREPARATION: current OFF pose -> screened roots/thumb/fingers -> hold -> OFF.'
    FOLDER_PREFIX='hand_prepare_current_empty'
    COMPLETE_MESSAGE='Empty hand preparation complete. Torque OFF verified; settings restored.'

    def validate_start(self,raw):
        a=super().validate_start(raw)
        if np.any(abs(a[:4]-THUMB_TARGET)>64):
            raise ValueError('Thumb differs from the reached preparation; no finger goals sent')
        return a

    def plan(self,raw):
        a=self.validate_start(raw)
        root=a.copy();root[ROOTS]=np.maximum(a[ROOTS],ROOT_TARGET)
        thumb=root.copy();thumb[:4]=THUMB_TARGET
        end=FULL_TARGET.copy()
        # Roots 8/12 never bend back below the accepted root clearance.
        end[[7,11]]=np.maximum(end[[7,11]],root[[7,11]])
        rows=[]
        for phase,b,count in [('stage_0_roots',root,128),
                              ('stage_1_thumb_prefix',thumb,64),
                              ('stage_2_fingers',end,FINGER_STEPS)]:
            for k in range(1,count+1):
                rows.append((phase,np.rint(a+(b-a)*k/count).astype(np.int64)))
            a=b.copy()
        return rows

    def frozen_for_phase(self,phase):
        if phase=='stage_0_roots':return np.array([i for i in range(16) if i not in ROOTS])
        if phase=='stage_1_thumb_prefix':return FROZEN
        return np.arange(4)

    def progress_for_phase(self,phase):
        if phase=='stage_1_thumb_prefix':return np.arange(4)
        if phase=='stage_2_fingers':return np.arange(4,16)
        return np.array([],dtype=int)
