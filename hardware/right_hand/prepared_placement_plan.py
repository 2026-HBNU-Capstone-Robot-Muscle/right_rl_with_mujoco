"""Recover the measured prepared hand for placement; all movement stays empty."""
import hashlib
import json
import numpy as np
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from thumb_live_start_plan import ROOT,CurrentPlan,ROOTS,ROOT_TARGET,THUMB_TARGET
from hand_preparation_plan import PreparationPlan,FULL_TARGET

SOURCE=ROOT/'hand_prepare_current_empty_20261022_004728_219622/result.json'
SOURCE_SHA='83eab4651771b60410ad8b45ffb2dd290e011e5ecf640a7d54e3c9556b2f828e'


def admission(raw):
    data=SOURCE.read_bytes()
    if hashlib.sha256(data).hexdigest()!=SOURCE_SHA:raise ValueError('Prepared-hand source changed')
    r=json.loads(data)
    if (r['status']!='empty_hand_preparation_complete' or not r['settings_restored']
            or not r['all_torque_off_verified']
            or r['joint_mapping']['motor_ids_in_sim_order']!=list(SIM_TO_MOTOR_IDS)):
        raise ValueError('Prepared-hand source invalid')
    known=np.array([r['initial']['position'],r['final_on_state']['position'],
                    FULL_TARGET,*[s['position'] for s in r['final_off_snapshots']]])
    a=np.asarray(raw)
    if a.shape!=(16,) or not np.isfinite(a).all() or np.any(a!=np.rint(a)):
        raise ValueError('Invalid current prepared-hand angles')
    # Measured endpoint envelope is admission only, not a clearance approval.
    # Actual current pose/path always receive a new native-model screen.
    if (np.any(a<0) or np.any(a>4095) or np.any(a<known.min(axis=0)-32)
            or np.any(a>known.max(axis=0)+32) or np.any(abs(a[:4]-THUMB_TARGET)>64)):
        raise ValueError('Current prepared posture requires a new review; no writes')
    return a.astype(np.int64)


def healthy_off(s):
    a=admission(s['position'])
    for key in ('torque','error','watchdog','voltage','temperature'):
        v=np.asarray(s[key])
        if v.shape!=(16,) or not np.isfinite(v).all() or np.any(v!=np.rint(v)):
            raise ValueError('Invalid OFF telemetry; no writes')
    if (any(np.any(s[k]) for k in ('torque','error','watchdog'))
            or np.any(np.asarray(s['voltage'])<38) or np.any(np.asarray(s['voltage'])>65)
            or np.any(np.asarray(s['temperature'])>45)):
        raise ValueError('Prepared OFF state unhealthy; no writes')
    return a


class PlacementPlan(PreparationPlan):
    PLAN_VERSION='prepared_hand_place_photo_remove_v1'
    EXTRA_PLANNER_FILES=('hand_preparation_plan.py','prepared_placement_plan.py')
    STATUS='prepared_placement_capture_complete'
    TITLE='CUBE PLACEMENT: EMPTY preparation -> PLACE/SPACE photo -> REMOVE/SPACE -> OFF.'
    FOLDER_PREFIX='prepared_placement_capture'
    COMPLETE_MESSAGE='Placement saved; cube removal confirmed. Torque OFF verified; settings restored.'
    CAPTURE_PLACEMENT=True
    validate_admission=staticmethod(admission)
    validate_off_state=staticmethod(healthy_off)

    def plan(self,raw):
        a=self.validate_start(raw)
        root=a.copy();root[ROOTS]=np.maximum(a[ROOTS],ROOT_TARGET)
        thumb=root.copy();thumb[:4]=THUMB_TARGET
        end=FULL_TARGET.copy();end[ROOTS]=np.maximum(end[ROOTS],root[ROOTS])
        rows=[]
        for phase,b in [('stage_0_roots',root),('stage_1_thumb_prefix',thumb),('stage_2_fingers',end)]:
            count=max(20,int(np.ceil(max(abs(b-a))/2)))
            for k in range(1,count+1):rows.append((phase,np.rint(a+(b-a)*k/count).astype(np.int64)))
            a=b.copy()
        return rows
