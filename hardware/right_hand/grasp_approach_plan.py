"""Fresh-start EMPTY approach check for a tentative tilted-cube grasp.

This plan has no placement callback and never permits a cube or learned action.
Every actual start is screened by the existing native-model gate before writes.
"""
import hashlib
import json
import numpy as np
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from thumb_live_start_plan import ROOT
from prepared_placement_plan import PlacementPlan

SOURCE=ROOT/'prepared_placement_capture_20261022_010440_125130/result.json'
SOURCE_SHA='c7b713d509e16623b7be3bd593aeee55fb8cb7f2676905b561a1a25a0098d029'
TARGET_FILE=ROOT/'prepared_cube_review_20261022/empty_grasp_target.json'
TARGET_SHA='7be1f3411e60c835b23080b44870b7c90fa2f6149160524c09ffceb4c5e7f598'
RING_FIXED=np.array([12,14,15])
RECOVERY_SOURCES=(
    ('grasp_approach_current_empty_20261022_015038_969787/result.json',
     '22523a5c7262ad0e0176b3b1036cc566b984c4cf29355bd2d4bf32f76d9f6107'),
    ('grasp_approach_current_empty_20261022_015140_792784/result.json',
     '1c2dcf439eae90567185598122a915dd1bcfd7b8b20e1ff617d039a6101a8258'))
PASSIVE_FINGER_JOINTS=np.array([5,6,9,10,13,14])


def target():
    data=TARGET_FILE.read_bytes()
    if hashlib.sha256(data).hexdigest()!=TARGET_SHA:raise ValueError('Grasp target changed; no writes')
    doc=json.loads(data)
    if doc['format']!='offline_tilted_cube_grasp_target_v1' or doc['hardware_execution_allowed']:
        raise ValueError('Invalid EMPTY source target')
    a=np.asarray(doc['target_ticks'])
    if a.shape!=(16,) or not np.isfinite(a).all() or np.any(a!=np.rint(a)):
        raise ValueError('Invalid grasp target angles')
    return a.astype(np.int64)


def admission(raw):
    data=SOURCE.read_bytes()
    if hashlib.sha256(data).hexdigest()!=SOURCE_SHA:raise ValueError('Latest placement source changed')
    r=json.loads(data)
    if (r['status']!='prepared_placement_capture_complete' or not r['settings_restored']
            or not r['all_torque_off_verified'] or not r['cube_removal_confirmed']
            or r['joint_mapping']['motor_ids_in_sim_order']!=list(SIM_TO_MOTOR_IDS)):
        raise ValueError('Latest placement source invalid')
    known=np.array([r['initial']['position'],r['placement_position_ticks'],r['placement_goal_ticks'],
                    *[s['position'] for s in r['final_off_snapshots']]])
    for index,(relative,digest) in enumerate(RECOVERY_SOURCES):
        data=(ROOT/relative).read_bytes()
        if hashlib.sha256(data).hexdigest()!=digest:raise ValueError('Reviewed OFF recovery source changed')
        trial=json.loads(data)
        expected=('stage_3_ring_clearance','preflight')[index]
        if (trial['status']!='stopped' or trial['failed_phase']!=expected
                or not trial['all_torque_off_verified']
                or trial['joint_mapping']['motor_ids_in_sim_order']!=list(SIM_TO_MOTOR_IDS)):
            raise ValueError('Invalid reviewed OFF recovery source')
        if index==0:
            if not trial['settings_restored']:raise ValueError('Recovery source did not restore settings')
            more=[trial['initial']['position'],*[s['position'] for s in trial['final_off_snapshots']]]
        else:
            if trial.get('write_events'):raise ValueError('Expected read-only recovery source')
            more=[trial['preflight_state']['position']]
        known=np.concatenate((known,np.array(more)))
    a=np.asarray(raw)
    if a.shape!=(16,) or not np.isfinite(a).all() or np.any(a!=np.rint(a)):
        raise ValueError('Invalid current prepared-hand angles')
    low=known.min(0)-32;high=known.max(0)+32
    # Finger rotation/PIP joints have repeatedly continued moving after OFF.
    # Their admission is bounded travel from the reached preparation (45deg),
    # not a claim that an entire envelope is clear. Fresh stable reads, native
    # limits, all 67 actual-start path cases and dynamics must still pass.
    preparation=np.asarray(r['placement_goal_ticks'])
    low[PASSIVE_FINGER_JOINTS]=preparation[PASSIVE_FINGER_JOINTS]-512
    high[PASSIVE_FINGER_JOINTS]=preparation[PASSIVE_FINGER_JOINTS]+512
    bad=(a<low)|(a>high)|(a<0)|(a>4095)
    if np.any(bad):
        i=int(np.flatnonzero(bad)[0])
        raise ValueError(f'Current posture outside reviewed recovery travel: ID{i+1}={a[i]}, allowed [{low[i]},{high[i]}]; no writes')
    target()  # A modified target cannot reach configuration/preload writes.
    return a.astype(np.int64)


def healthy_off(s):
    a=admission(s['position'])
    for k in ('torque','error','watchdog','voltage','temperature'):
        v=np.asarray(s[k])
        if v.shape!=(16,) or not np.isfinite(v).all() or np.any(v!=np.rint(v)):
            raise ValueError('Invalid OFF health telemetry; no writes')
    if (any(np.any(s[k]) for k in ('torque','error','watchdog')) or np.any(np.asarray(s['voltage'])<38)
            or np.any(np.asarray(s['voltage'])>65) or np.any(np.asarray(s['temperature'])>45)):
        raise ValueError('Unhealthy OFF state; no writes')
    return a


class GraspApproachPlan(PlacementPlan):
    PLAN_VERSION='empty_tilted_cube_grasp_approach_v2'
    EXTRA_PLANNER_FILES=('hand_preparation_plan.py','prepared_placement_plan.py','grasp_approach_plan.py',
                         'prepared_cube_review_20261022/empty_grasp_target.json',
                         'held_joint_guard.py','leap_thumb_prefix_empty.py')
    STATUS='empty_grasp_approach_complete'
    TITLE='EMPTY GRASP APPROACH v2: settling-aware hold check; fresh OFF recovery -> new approach -> OFF.'
    FOLDER_PREFIX='grasp_approach_current_empty'
    COMPLETE_MESSAGE='Empty grasp approach complete. OFF/settings restored. Cube grasp and rotation are NOT verified.'
    CAPTURE_PLACEMENT=False
    HELD_JOINT_GUARD=True
    validate_admission=staticmethod(admission)
    validate_off_state=staticmethod(healthy_off)

    def plan(self,raw):
        rows=super().plan(raw)
        a=rows[-1][1].copy();b=target()
        # The ring's other three joints retain the prepared command, including
        # its freshly screened root clearance; no deliberate return into wood.
        b[RING_FIXED]=a[RING_FIXED]
        relief=a.copy();relief[13]=b[13]
        for phase,end in [('stage_3_ring_clearance',relief),('stage_4_grasp_approach',b)]:
            n=max(20,int(np.ceil(np.max(abs(end-a))/2)))
            for k in range(1,n+1):rows.append((phase,np.rint(a+(end-a)*k/n).astype(np.int64)))
            a=end.copy()
        return rows

    def frozen_for_phase(self,phase):
        if phase=='stage_3_ring_clearance':return np.array([i for i in range(16) if i!=13])
        if phase in ('stage_4_grasp_approach','stage_2_fixed_hold'):return RING_FIXED
        return super().frozen_for_phase(phase)

    def progress_for_phase(self,phase):
        if phase=='stage_3_ring_clearance':return np.array([13])
        if phase=='stage_4_grasp_approach':return np.arange(12)
        return super().progress_for_phase(phase)
