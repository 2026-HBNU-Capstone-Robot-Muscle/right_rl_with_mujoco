"""Empty recovery, placement pause, then the completed empty grasp approach.

The source is a user-run EMPTY trial, not a verified grasp. Real starts still
need the independent native path screen. No rotation or learned action exists
in this plan, and the loaded segment requires an explicit placement checkpoint.
"""
import hashlib
import json
import numpy as np
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from thumb_live_start_plan import ROOT,CurrentPlan
from grasp_recovery_plan import empty_opening,interpolate

SOURCE=ROOT/'grasp_approach_current_empty_20261022_020751_741219/result.json'
SOURCE_SHA='72d12e547078ef534fe55d9b5b87d7442e94e8cee90c403b8f173516f0a5fc07'
LOADED_PHASE='stage_3_loaded_approach'
MAX_EMPTY_RECOVERY_TICKS=512  # <=45deg, with every actual path independently screened.


def completed_source():
    data=SOURCE.read_bytes()
    if hashlib.sha256(data).hexdigest()!=SOURCE_SHA:raise ValueError('Completed empty approach changed; no writes')
    r=json.loads(data)
    if (r['status']!='empty_grasp_approach_complete' or not r['all_torque_off_verified']
            or not r['settings_restored'] or r['policy_used'] or r['physical_flip_verified']
            or r['joint_mapping']['motor_ids_in_sim_order']!=list(SIM_TO_MOTOR_IDS)):
        raise ValueError('Invalid completed empty approach source')
    return r


def endpoints():
    r=completed_source()
    first=next(s for s in r['samples'] if s['phase']=='stage_4_grasp_approach')
    opened=np.asarray(first['goal'],dtype=np.int64)
    closed=np.asarray(r['final_on_state']['goal'],dtype=np.int64)
    if opened.shape!=(16,) or closed.shape!=(16,) or np.any(opened[12:]!=closed[12:]):
        raise ValueError('Completed approach does not hold ring goals')
    return opened,closed


def admission(raw):
    r=completed_source();a=np.asarray(raw)
    if a.shape!=(16,) or not np.isfinite(a).all() or np.any(a!=np.rint(a)):
        raise ValueError('Invalid raw start; no writes')
    # OFF relaxation is not a historical pose-lock. Bound the movement needed
    # to return to the reached goal instead. This is admission to simulation,
    # never clearance approval: stable fresh reads, native limits/re-entry,
    # 67 exact-start paths and dynamics must all pass before any setting write.
    opened,closed=endpoints()
    # A stop during opening must also remain recoverable. Admit bounded travel
    # from either reached waypoint, then screen the entire exact new route.
    candidate=opened if max(abs(a-opened))<max(abs(a-closed)) else closed
    delta=abs(a-candidate)
    bad=(delta>MAX_EMPTY_RECOVERY_TICKS)|(a<0)|(a>4095)
    if np.any(bad):
        i=int(np.flatnonzero(bad)[0])
        raise ValueError(f'Empty recovery travel exceeds {MAX_EMPTY_RECOVERY_TICKS} ticks: ID{i+1}={a[i]}, nearest waypoint={candidate[i]}; no writes')
    rows=empty_opening(a,opened);interpolate(rows,opened,closed,LOADED_PHASE)
    if len(rows)*.05+2>50:
        raise ValueError('Measured opening path exceeds the 50s planning budget; no writes')
    return a.astype(np.int64)


def healthy_off(s):
    a=admission(s['position'])
    for k in ('torque','error','watchdog','voltage','temperature'):
        v=np.asarray(s[k])
        if v.shape!=(16,) or not np.isfinite(v).all() or np.any(v!=np.rint(v)):
            raise ValueError('Invalid OFF health telemetry; no writes')
    if (any(np.any(s[k]) for k in ('torque','error','watchdog'))
            or np.any(np.asarray(s['voltage'])<38) or np.any(np.asarray(s['voltage'])>65)
            or np.any(np.asarray(s['temperature'])>45)):
        raise ValueError('Unhealthy OFF state; no writes')
    return a


class GraspPlacePlan(CurrentPlan):
    PLAN_VERSION='cube_place_then_completed_approach_v6'
    EXTRA_PLANNER_FILES=('grasp_place_plan.py','grasp_place_capture.py','held_joint_guard.py',
                         'leap_thumb_prefix_empty.py','leap_thumb_prepare_current_empty.py',
                         'initial_contact_escape.py','grasp_recovery_plan.py',
                         'sequence_flip_checked.py','goal_readback_confirmation.py',
                         'thumb_live_start_gate.py','empty_trial_camera.py',
                         'camera_frame_signal.py','camera_stream.py')
    STATUS='cube_grasp_approach_trial_complete'
    TITLE='CUBE GRASP TRIAL v6: ring release/CONFIRM -> OPEN -> PLACE/SPACE -> approach -> REMOVE/SPACE -> OFF.'
    CONFIRM_PREVIOUS_GOAL_READBACK=True
    FOLDER_PREFIX='cube_grasp_trial'
    COMPLETE_MESSAGE='Loaded approach observation complete; removal confirmed; OFF/settings restored. Rotation NOT tested.'
    CAPTURE_PLACEMENT=True
    CAPTURE_BEFORE_PHASE=LOADED_PHASE
    PLACEMENT_PAUSE_LIMIT=100
    CHECKPOINT_SETTLE_SAMPLES=20
    CHECKPOINT_MAX_ERROR_TICKS=64
    CONTACT_RELEASE_BEFORE_PHASE='stage_1_thumb_clearance'
    INITIAL_SELF_ESCAPE_GOALS=40
    # The existing 40-goal separation window, at <=2 ticks per goal, bounds
    # first separation to <=80 PIP ticks. The former extra 64-tick cutoff
    # rejected a 74-tick/37-goal monotone release while every other static
    # and dynamic check passed. This changes no generated movement targets.
    MAX_CONTACT_RELEASE_TRAVEL_TICKS=INITIAL_SELF_ESCAPE_GOALS*CurrentPlan.MAX_GOAL_STEP
    CONTACT_RELEASE_MARGIN_TICKS=16
    HELD_JOINT_GUARD=True
    # Passive OFF sag can place the thumb just beyond the native model range.
    # Do not widen any range. Only the first EMPTY segment may monotonically
    # return from <=0.06rad existing excess to zero before placement/motion.
    NATIVE_REENTRY_PHASE='stage_0_empty_recover'
    MAX_START_RANGE_EXCESS_RAD=.06
    # The newly measured OFF ring has settled toward the palm. Only these
    # existing tip/palm contacts may be separated, never deepened or created.
    INITIAL_SELF_ESCAPE_PAIRS=('palm_collision_4|rf_tip','palm_collision_6|rf_tip',
                              'palm_collision_7|rf_tip')
    # An approximate collision primitive overlap is not a measured pressure
    # depth. Use a short, monotone, <=40-goal geometric separation certificate,
    # measured release threshold and an operator observation instead. These
    # fields cannot enable motion without the separately verified certificate.
    MAX_INITIAL_SELF_ESCAPE_MM=None
    INITIAL_SELF_ESCAPE_DYNAMIC_S=2.5  # includes 0.5s initial modeled holding.
    validate_admission=staticmethod(admission)
    validate_off_state=staticmethod(healthy_off)

    def plan(self,raw):
        a=self.validate_start(raw);opened,closed=endpoints();rows=[]
        rows=empty_opening(a,opened)
        interpolate(rows,opened,closed,LOADED_PHASE)
        return rows

    def frozen_for_phase(self,phase):
        if phase=='stage_0_empty_recover':return np.array([i for i in range(16) if i not in (3,14)])
        if phase=='stage_1_thumb_clearance':return np.arange(4,16)
        if phase in ('release_checkpoint','placement_checkpoint','stage_2_fixed_hold'):return np.arange(16)
        if phase==LOADED_PHASE:return np.arange(12,16)
        return np.array([],dtype=int)

    def progress_for_phase(self,phase):
        if phase=='stage_0_empty_recover':return np.array([3,14])
        if phase=='stage_1_thumb_clearance':return np.arange(4)
        if phase in ('release_checkpoint','placement_checkpoint'):return np.array([],dtype=int)
        return np.arange(12) if phase==LOADED_PHASE else np.arange(16)

    def screen_dynamic_path(self,raw):
        rows=[];previous=None
        for phase,q in self.plan(raw):
            if phase==LOADED_PHASE and previous is not None:
                rows.extend([('placement_hold',previous.copy())]*40)
            rows.append((phase,q))
            previous=None if phase==LOADED_PHASE else q
        return rows
