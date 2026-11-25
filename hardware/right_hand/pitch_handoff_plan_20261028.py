"""Observe the demonstrated pitch followed by a recorded, screened hand transfer.

No subsequent yaw is queued. Only a user-started operator can execute targets.
"""
import hashlib,json
from pathlib import Path
import numpy as np
from calibrated_quarter_plan_20261028 import HorizontalQuarterPlan as BasePlan,hashes as base_hashes
from calibrated_empty_ring_margin_20261028 import rows as margin_rows,interpolate
from new_pair_grasp_plan_20261027 import ENTRY,ROOTS,OPEN,SETTLE
from horizontal_quarter_plan_20261027 import PREFIX_PHASES as ORIGINAL_PREFIX,OPEN_PATH
from loaded_quarter_reentry_20261025 import validate_path

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261028/two_axis_connection/operator_pitch_handoff_v3_side_clearance'
REQUEST=OUT/'request.json';PROOF=OUT/'review.json'
LOADED_PATH=OUT/'loaded_path.npz'
PITCH=ROOT/'right_hand_rotation_20261028/two_axis_connection/saved_pitch/loaded_pitch_path.npz'
SOURCE_REVIEW=PITCH.with_name('source_review.json')
TRANSFER=ROOT/'right_hand_rotation_20261028/two_axis_connection/hand_join/together.npz'
PREFIX_PHASES=(*ORIGINAL_PREFIX,'stage_0_ring_pip_margin')
PITCH_PHASE='stage_5_known_pitch_rf_clearance'
PITCH_HOLD='stage_6_pitch_hold'
TRANSFER_PHASE='stage_7_loaded_pose_transfer'
TRANSFER_HOLD='stage_8_transfer_observe_hold'
RING_SIDE_DELTA=8
INDEX_SIDE_DELTA=-8
TRANSFER_PATH=OUT/'transfer_path.npz'
TRANSFER_SEED=ROOT/'right_hand_rotation_20261026/yaw_after_flip_v1/seed.json'
FILES=('pitch_handoff_plan_20261028.py','screen_pitch_handoff_empty_20261028.py',
       'screen_pitch_handoff_loaded_20261028.py','pitch_handoff_gate_20261028.py',
       'pitch_handoff_capture_20261028.py','leap_pitch_handoff_20261028.py','leap_pitch_handoff_observe.cmd',
       'screen_pitch_handoff_review_20261028.py','screen_pitch_handoff_transfer_20261028.py',
       'replay_pitch_yaw_join_20261028.py','pitch_handoff_offscreen_process_20261028.py',
       'slow_thumb_seam_resolution_20261026.py')

def pitch_start():
    a=np.load(PITCH)['start'].copy();a[5]+=INDEX_SIDE_DELTA;a[13]+=RING_SIDE_DELTA;return a

def loaded_rows():
    p=np.load(PITCH);transfer=np.load(TRANSFER)
    np.testing.assert_array_equal(p['goals'][-1],transfer['start'])
    np.testing.assert_array_equal(transfer['goals'][-1],np.load(OPEN_PATH)['open_goal'])
    # Preserve the other14 recorded controls and the75ms schedule. The two
    # axial joints each move0.703deg for explicitly screened side clearance.
    revised=p['goals'].copy();revised[:,5]+=INDEX_SIDE_DELTA;revised[:,13]+=RING_SIDE_DELTA
    rows=[(PITCH_HOLD if str(phase)=='stage_11_loaded_new_pose_hold' else PITCH_PHASE,q.copy())
          for phase,q in zip(p['phases'],revised)]
    moved=interpolate(revised[-1],transfer['goals'][-1])
    if len(moved)!=len(transfer['goals']):raise ValueError('Transfer timing changed')
    rows.extend((TRANSFER_PHASE,q.copy()) for q in moved)
    rows.extend((TRANSFER_HOLD,moved[-1].copy()) for _ in range(40))
    return rows

def prefix_rows(plan,raw):
    rows=margin_rows(plan,raw)
    start=pitch_start()
    rows.extend((OPEN,q) for q in interpolate(rows[-1][1],start))
    rows.extend((SETTLE,start.copy()) for _ in range(20))
    return rows

def prefix_digest(plan):
    rows=prefix_rows(plan,plan.source_position())
    return hashlib.sha256(json.dumps([(p,q.tolist()) for p,q in rows],separators=(',',':')).encode()).hexdigest()

def hashes():
    paths=[ROOT/name for name in FILES]+[PITCH,SOURCE_REVIEW,TRANSFER,LOADED_PATH,TRANSFER_PATH,TRANSFER_SEED,REQUEST,
        OUT/'loaded_geometry/review.json',OUT/'loaded_hand_dynamics/review.json',OUT/'transfer_retention/review.json']
    return {**base_hashes(),**{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}}

class PitchHandoffPlan(BasePlan):
    PLAN_VERSION='pitch_if6_minus8_rf14_plus8_then_transfer_no_yaw_v3_20261028'
    TITLE='PITCH/HANDOFF OBSERVATION: EMPTY setup -> PLACE -> known pitch -> transfer -> REMOVE -> OFF. No yaw.'
    FOLDER_PREFIX='pitch_handoff_observe';STATUS='pitch_handoff_observation_complete'
    COMPLETE_MESSAGE='Known pitch and hand-transfer observation complete; cube removed; torque OFF/settings restored. Review video before any next rotation.'
    # This is an aggregate displacement guard, not a joint-angle limit. The
    # two-setup observation visits the previously demonstrated3349 CMC goal
    # from the newly read1798 OFF position (1551 ticks). Its own small fixed
    # bound covers the screened +/-4 start/enable neighborhood. The old yaw
    # experiment's1300 bound and every configured native joint range remain
    # unchanged; entry is still monotonic from the existing passive corridor.
    MAX_THUMB_TRAVEL=1568
    CAPTURE_BEFORE_PHASE=PITCH_PHASE
    proof_hashes=staticmethod(hashes)
    def __init__(self,request_path):
        super().__init__(request_path)
        self.raw_off_ticks=self.center.copy();self.review_center_reused=False
        # The model review already admits a4-tick start/enable neighborhood.
        # Retain that reviewed centre for fresh ADC noise within it, rather
        # than demanding byte-identical OFF readings and rerunning the model.
        # Fresh OFF health was validated above; live and enabled starts must
        # still pass the original4-tick check against this same centre.
        if self.SOURCE.resolve()!=REQUEST.resolve() and REQUEST.is_file():
            reference=np.asarray(json.loads(REQUEST.read_bytes())['off_state']['position'],dtype=int)
            if np.max(abs(self.raw_off_ticks-reference))<=4:
                self.center=reference;self.review_center_reused=True
    def plan(self,raw):
        a=self.validate_start(raw);rows=prefix_rows(self,raw)+loaded_rows()
        values=validate_path(a,rows,ENTRY)
        if np.max(abs(values[:,:4]-a[:4]))>self.MAX_THUMB_TRAVEL:raise ValueError('Existing thumb-travel bound exceeded')
        if (len(rows)+40)*self.SAMPLE_PERIOD+20>self.ON_TIMEOUT:raise ValueError('Existing300s active-time bound exceeded')
        return rows
    def frozen_for_phase(self,phase):
        if phase in (PITCH_PHASE,TRANSFER_PHASE):return np.array([],dtype=int)
        if phase in (PITCH_HOLD,TRANSFER_HOLD):return np.arange(16)
        return super().frozen_for_phase(phase)
    def progress_for_phase(self,phase):
        if phase in (PITCH_PHASE,TRANSFER_PHASE):return np.arange(16)
        if phase in (PITCH_HOLD,TRANSFER_HOLD):return np.array([],dtype=int)
        return super().progress_for_phase(phase)
