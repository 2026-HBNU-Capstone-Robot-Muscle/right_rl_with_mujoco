"""Bounded first physical yaw observation after the completed grasp approach.

Fixed offline CEM trajectory, not a learned SAC action. Clearance passed in 77
provisional model cases; the requested ten-degree rotation did NOT pass all.
Every fresh real OFF start still needs the full independent native screen.
"""
import hashlib
import json
import numpy as np
from grasp_place_plan import GraspPlacePlan, admission as grasp_admission, endpoints
from grasp_recovery_plan import empty_opening, interpolate
from thumb_live_start_plan import ROOT
from cube_small_rotation_capture import SmallRotationCapture

ROTATION_PHASE='stage_4_small_rotation'
CANDIDATE=ROOT/'right_hand_rotation_20261022/small_rotation_trial_candidate.json'
CANDIDATE_SHA='c34c63666c2a4835c96e47d97a522b59e09534e482fba467b35a34d54a316c95'


def candidate():
    blob=CANDIDATE.read_bytes()
    if hashlib.sha256(blob).hexdigest()!=CANDIDATE_SHA:raise ValueError('Rotation candidate changed; no writes')
    r=json.loads(blob)
    q=np.asarray(r['goal_ticks']);_,closed=endpoints()
    if (r['version']!='bounded_small_yaw_commissioning_v1' or r['live_sac'] is not False
            or r['physical_rotation_verified'] is not False or r['sample_period_s']!=.05
            or r['clearance_tracking_cases_passed']!=77 or q.shape!=(80,16)
            or not np.isfinite(q).all() or np.any(q!=np.rint(q))
            or r['base_goal_ticks']!=closed.tolist() or r['pre_rotation_hold_samples']!=0
            or np.max(abs(np.diff(np.vstack([closed,q]),axis=0)))>2
            or np.max(abs(q-closed))>96 or np.any(q[:,12:]!=closed[12:])):
        raise ValueError('Invalid bounded rotation candidate; no writes')
    replay=ROOT/'right_hand_rotation_20261022/small_yaw_refined_result_independent.json'
    if hashlib.sha256(replay.read_bytes()).hexdigest()!=r['independent_replay_sha256']:
        raise ValueError('Independent rotation replay changed; no writes')
    return q.astype(np.int64)


def admission(raw):
    a=grasp_admission(raw);opened,closed=endpoints()
    rows=empty_opening(a,opened);interpolate(rows,opened,closed,'stage_3_loaded_approach')
    if (len(rows)+80)*.05+2>50:
        raise ValueError('Preparation plus rotation exceeds the 50s planning budget; no writes')
    candidate()
    return a


def healthy_off(s):
    # Preserve all existing telemetry checks, then include the longer budget.
    GraspPlacePlan.validate_off_state(s)
    return admission(s['position'])


class SmallRotationPlan(GraspPlacePlan):
    PLAN_VERSION='cube_first_small_yaw_observation_v1'
    STATUS='cube_small_rotation_path_complete'
    TITLE='SMALL CUBE ROTATION TRIAL: EMPTY opening -> PLACE -> grasp -> small yaw -> REMOVE -> OFF.'
    COMPLETE_MESSAGE='Small rotation command path complete; removal confirmed; OFF/settings restored. Actual cube rotation needs observation.'
    FOLDER_PREFIX='cube_small_rotation_trial'
    CAPTURE_PROTOCOL_TYPE=SmallRotationCapture
    EXTRA_PLANNER_FILES=GraspPlacePlan.EXTRA_PLANNER_FILES+(
        'cube_small_rotation_plan.py','cube_small_rotation_capture.py',
        'right_hand_rotation_20261022/small_rotation_trial_candidate.json',
        'right_hand_rotation_20261022/small_yaw_refined_result_independent.json',
        'leap_cube_small_rotation_trial.py')
    validate_admission=staticmethod(admission)
    validate_off_state=staticmethod(healthy_off)

    def plan(self,raw):
        rows=super().plan(raw)
        rows.extend((ROTATION_PHASE,q.copy()) for q in candidate())
        return rows

    def frozen_for_phase(self,phase):
        if phase==ROTATION_PHASE:return np.arange(12,16)
        return super().frozen_for_phase(phase)

    def progress_for_phase(self,phase):
        if phase==ROTATION_PHASE:return np.arange(12)
        return super().progress_for_phase(phase)
