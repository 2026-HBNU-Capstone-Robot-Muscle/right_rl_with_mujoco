"""Small motion from the completed grasp; fresh native proof remains mandatory."""
import hashlib
import json
import numpy as np
import read_recovery_confirmation  # Load while OFF; the ON sample call uses the cached module.
from slow_grasp_demo_plan import SlowGraspDemoPlan,OBSERVATION_SAMPLES,demo_opening
from grasped_yaw_capture import GraspedYawCapture
from thumb_live_start_plan import ROOT
from marker_trial_camera import MarkerTrialCamera
from grasp_recovery_plan import interpolate

CANDIDATE=ROOT/'right_hand_rotation_20261024/contact_refined_small_yaw_candidate.json'
CANDIDATE_SHA='e16ee60f16bf36c4d9f79dedf4118689fb6ebff40e03d010cabe575ef14c76bd'
CANDIDATE_VERSION='contact_refined_small_yaw_candidate_v1'
CANDIDATE_SOURCES=(
    'grasped_yaw_trial_20261023_234707_886202/result.json',
    'right_hand_rotation_20261024/contact_point_yaw_search.json',
    'right_hand_rotation_20261024/completed_yaw_active_cube_replay.json',
    'slow_grasp_demo_20261023_182105_573825/on_paired_pose_review.json',
    'right_hand_rotation_20261023/native_motor_limits.json')
TURN_PHASE='stage_5_grasped_small_yaw'
AFTER_PHASE='stage_6_yaw_fixed_observation'
RING_CLEARANCE_PHASE='stage_1_ring_clearance'
TURN_SAMPLES=90
AFTER_SAMPLES=40
FROZEN_MOTORS=np.array([4,8,12,13,14,15,16])-1


def yaw_opening(raw,opened):
    """Finish a released ring's return before thumb/finger paths cross it.

    The existing palm-release goals are preserved. Only after their operator
    checkpoint may the four ring joints go to the already reached open pose.
    Other starts retain their old route plus a 20-sample fixed checkpoint.
    """
    original=demo_opening(raw,opened)
    release=[(phase,q.copy()) for phase,q in original if phase=='stage_0_empty_recover']
    a=release[-1][1].copy()
    tip_released=int(a[12])!=int(raw[12])
    ring=a.copy()
    if tip_released:ring[12:]=opened[12:]
    rows=release
    interpolate(rows,a,ring,RING_CLEARANCE_PHASE)
    for phase,q in original:
        if phase=='stage_0_empty_recover':continue
        q=q.copy()
        if tip_released:q[12:]=opened[12:]
        rows.append((phase,q))
    return rows


class GraspedYawCamera(MarkerTrialCamera):
    # The operator's 22:03 same-scene sweep retained ID5 in all six frames at
    # -5/-6; -5 visibly exposes the black hand better. Palm glare persists,
    # so this change does not certify registration or require ID40 while ON.
    EXPOSURE=-5.


class GraspedYawPlan(SlowGraspDemoPlan):
    PLAN_VERSION='same_completed_grasp_small_yaw_observation_v1'
    STATUS='grasped_small_yaw_path_complete'
    FOLDER_PREFIX='grasped_yaw_trial'
    TITLE='CONTACT-REFINED SMALL YAW: EMPTY preparation -> PLACE -> grasp -> fixed 3s -> contact refinement/small turn -> fixed 2s -> REMOVE -> OFF.'
    COMPLETE_MESSAGE='Small-turn commands and before-removal observation complete. OFF/settings restored; physical rotation needs review.'
    CAPTURE_PROTOCOL_TYPE=GraspedYawCapture
    CAMERA_TYPE=GraspedYawCamera
    ON_TIMEOUT=75
    CONFIRM_FRESH_AFTER_READ_RECOVERY=True
    CONTACT_RELEASE_BEFORE_PHASE=RING_CLEARANCE_PHASE
    EXTRA_PLANNER_FILES=SlowGraspDemoPlan.EXTRA_PLANNER_FILES+(
        'grasped_yaw_plan.py','grasped_yaw_capture.py','leap_grasped_yaw_trial.py','read_recovery_confirmation.py',
        'prepare_contact_point_yaw_20261024.py','export_contact_yaw_candidate_20261024.py',
        'replay_completed_yaw_cube_contact_20261024.py',
        'right_hand_rotation_20261024/contact_refined_small_yaw_candidate.json',
        'slow_grasp_demo_20261023_182105_573825/result.json',
        )+CANDIDATE_SOURCES

    def __init__(self,request):
        super().__init__(request)
        blob=CANDIDATE.read_bytes()
        if hashlib.sha256(blob).hexdigest()!=CANDIDATE_SHA:raise ValueError('Small-turn candidate changed; no writes')
        c=json.loads(blob);q=np.asarray(c['goal_ticks'])
        if (c['version']!=CANDIDATE_VERSION or c['live_sac'] is not False
            or c['physical_cube_rotation_verified'] is not False or c['joint_offsets_fitted'] is not False
            or c['experimental_trial_only'] is not True or c['robust_policy'] is not False
            or c['full_held_out_angle_criterion_passed'] is not False
            or c['selected']['held_out_pass'] is not False
            or c['evaluated_direction_consistent'] is not True
            or c['marker_to_model_registration_verified'] is not False
            or c['current_or_PID_changed'] is not False or c['hardware_execution_allowed'] is not False
            or c['desired_physical_yaw_degrees']!=3. or c['kinematic_angle_parameter_degrees']!=-3.
            or c['contact_refinement_samples']!=20 or c['rotation_samples']!=70
            or q.shape!=(TURN_SAMPLES,16)
            or not np.isfinite(q).all() or np.any(q!=np.rint(q))
            or c['base_goal_ticks']!=self._closed.tolist()
            or np.max(abs(q-self._closed))>64
            or np.max(abs(np.diff(np.vstack([self._closed,q]),axis=0)))>2
            or np.any(q[:,FROZEN_MOTORS]!=self._closed[FROZEN_MOTORS])
            or np.max(abs(q[:20]-self._closed))>18
            or (q[19]-self._closed).tolist()!=c['closure_delta_ticks']
            or np.any(q<self._lower) or np.any(q>self._upper)):
            raise ValueError('Invalid bounded small-turn candidate; no writes')
        if set(c['source_hashes'])!=set(CANDIDATE_SOURCES):
            raise ValueError('Small-turn source evidence list changed; no writes')
        for relative in CANDIDATE_SOURCES:
            if hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()!=c['source_hashes'][relative]:
                raise ValueError('Small-turn source evidence changed; no writes')
        self._yaw=q.astype(np.int64).copy();self._yaw.flags.writeable=False
        changing=np.ptp(np.vstack([self._closed,q]),axis=0)>0
        self._yaw_frozen=np.flatnonzero(~changing);self._yaw_progress=np.flatnonzero(changing)

    def validate_start(self,raw):
        a=super().validate_start(raw)
        prefix=len(yaw_opening(a,self._opened))+max(20,int(np.ceil(max(abs(self._closed-self._opened))/2)))+OBSERVATION_SAMPLES
        if (prefix+TURN_SAMPLES+AFTER_SAMPLES+20)*.06+2>self.ON_TIMEOUT:
            raise ValueError('Small-turn path exceeds the 75s delayed-cycle budget; no writes')
        return a

    def plan(self,raw):
        rows=super().plan(raw)
        # Keep every loaded grasp/hold waypoint intact. Replace only the EMPTY
        # prefix; the release operator check now precedes ring clearance.
        suffix=[(phase,q) for phase,q in rows if phase in ('stage_3_loaded_approach','stage_4_fixed_grasp_observation')]
        rows=yaw_opening(raw,self._opened)+suffix
        rows.extend((TURN_PHASE,q.copy()) for q in self._yaw)
        rows.extend((AFTER_PHASE,self._yaw[-1].copy()) for _ in range(AFTER_SAMPLES))
        return rows

    def frozen_for_phase(self,phase):
        if phase==RING_CLEARANCE_PHASE:return np.arange(12)
        if phase==TURN_PHASE:return self._yaw_frozen.copy()
        if phase==AFTER_PHASE:return np.arange(16)
        return super().frozen_for_phase(phase)

    def progress_for_phase(self,phase):
        if phase==RING_CLEARANCE_PHASE:return np.arange(12,16)
        if phase==TURN_PHASE:return self._yaw_progress.copy()
        if phase==AFTER_PHASE:return np.array([],dtype=int)
        return super().progress_for_phase(phase)
