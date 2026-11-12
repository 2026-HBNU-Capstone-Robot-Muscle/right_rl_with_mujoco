"""Isolated638-goal yaw calibration, with reviewed automatic opening."""
import hashlib,json
import numpy as np
from horizontal_quarter_plan_20261027 import HorizontalQuarterPlan as BasePlan,ROOT,PREFIX_PHASES as BASE_PREFIX_PHASES,PHASE_MAP,OPEN_PATH,ENTRY,serial_roots,validate_path,hashes as base_hashes
from new_pair_grasp_plan_20261027 import NewPairGraspPlan
from calibrated_empty_ring_margin_20261028 import PHASE as RING_MARGIN,rows as margin_rows
PREFIX_PHASES=(*BASE_PREFIX_PHASES,RING_MARGIN)
OUT=ROOT/'right_hand_rotation_20261028/observed90_prefix638/operator_review_v4_empty_margin'
REQUEST=OUT/'request.json';PROOF=OUT/'review.json'
DATA=ROOT/'right_hand_rotation_20261028/observed90_prefix638'
LOADED_PATH=DATA/'full_loaded_27/loaded_path.npz'
LOADED_REVIEW=DATA/'full_loaded_27/review.json'
STATIC_REVIEW=DATA/'static_201/review.json'
def hashes():
    names=('calibrated_quarter_plan_20261028.py','calibrated_empty_ring_margin_20261028.py','screen_calibrated_empty_margin_20261028.py','calibrated_precise_clock_20261028.py','precise_two_full_readonly_20261028_060908_257593/result.json','calibrated_quarter_trial_20261028_055850_634642/timing_review.json','calibrated_precise_window_20261028.py','calibrated_precise_receive_20261028.py','calibrated_two_full_health_20261028.py','horizontal_single_reply40_20261028.py','single_reply40_evidence_20261028.py','single_reply_profile_20261028_054656_033637/result.json','calibrated_quarter_source_20261028.py','calibrated_quarter_gate_20261028.py','leap_calibrated_quarter_20261028.py','leap_calibrated_quarter_trial.cmd','screen_calibrated_quarter_20261028.py',
      'right_hand_rotation_20261028/observed90_prefix638/candidate.npz','right_hand_rotation_20261028/observed90_prefix638/calibration_source.json',
      'right_hand_rotation_20261028/completed_quarter_045326/rotation_trace.json','trace_observed_rotation_045326_20261028.py',
      'horizontal_quarter_trial_20261028_045326_636152/result.json','horizontal_quarter_trial_20261028_045326_636152/camera/frame_01445.jpg')
    return {**base_hashes(),**{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in [*(ROOT/n for n in names),LOADED_PATH,LOADED_REVIEW,STATIC_REVIEW]}}
def loaded_rows():
    z=np.load(LOADED_PATH)
    if not np.array_equal(z['start'],np.load(OPEN_PATH)['open_goal']) or not np.array_equal(z['goals'][-1],z['start']):raise ValueError('Calibration does not join reviewed OPEN pose')
    return [(PHASE_MAP[str(p)],q.copy()) for p,q in zip(z['phases'],z['goals'])]
def prefix_digest(plan):
    rows=[(p,q.tolist()) for p,q in plan.plan(plan.source_position()) if p in PREFIX_PHASES]
    return hashlib.sha256(json.dumps(rows,separators=(',',':')).encode()).hexdigest()
class HorizontalQuarterPlan(BasePlan):
    PLAN_VERSION='physical_timing_prefix638_calibration_v4_empty_margin_20261028'
    TITLE='CALIBRATED SHORTER TURN: EMPTY -> PLACE -> grasp ->638 yaw goals -> HOLD -> OPEN -> REMOVE -> OFF.'
    FOLDER_PREFIX='calibrated_quarter_trial';STATUS='calibrated_quarter_observation_complete'
    COMPLETE_MESSAGE='Shortened rotation observation complete; cube removed; OFF/settings restored. Actual90-degree angle still requires video review.'
    proof_hashes=staticmethod(hashes)
    def plan(self,raw):
        a=self.validate_start(raw)
        prefix=margin_rows(self,raw)
        rows=prefix+loaded_rows();values=validate_path(a,rows,ENTRY)
        if (len(rows)+40)*self.SAMPLE_PERIOD+20>self.ON_TIMEOUT:raise ValueError('Calibration exceeds bounded active time')
        if np.max(abs(values[:,:4]-a[:4]))>self.MAX_THUMB_TRAVEL:raise ValueError('Thumb travel exceeds existing bound')
        return rows

    def frozen_for_phase(self,phase):
        if phase==RING_MARGIN:return np.array([i for i in range(16) if i!=14])
        return super().frozen_for_phase(phase)
    def progress_for_phase(self,phase):
        if phase==RING_MARGIN:return np.array([14])
        return super().progress_for_phase(phase)
