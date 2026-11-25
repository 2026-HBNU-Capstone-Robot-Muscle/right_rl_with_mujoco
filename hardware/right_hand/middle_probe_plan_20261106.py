"""Supervised middle-finger clearance measurement with an unchanged thumb."""
import hashlib,json
from pathlib import Path
import numpy as np
from natural_cradle_grasp_plan_20261030 import NaturalCradleGraspPlan,validate_candidate as natural_candidate,hashes as natural_hashes
from calibrated_empty_ring_margin_20261028 import rows as prefix_rows
from loaded_quarter_reentry_20261025 import limits,validate_path
from new_pair_grasp_plan_20261027 import ENTRY
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261106/middle_clearance_probe'
REQUEST=O/'request.json';PROOF=O/'review_centered_v2.json';CANDIDATE=O/'candidate.npz';ALIAS=O/'evaluation_alias.npz'
FOLD='stage_7_middle_fold';HOLD='stage_8_middle_hold';RETURN='stage_8b_middle_return';RESTORED='stage_8c_restored_hold'
MIDDLE=np.arange(8,12);FIXED=np.r_[np.arange(8),np.arange(12,16)]
FILES=('middle_probe_plan_20261106.py','middle_probe_gate_20261106.py','middle_probe_capture_20261106.py','leap_middle_clearance_probe_20261106.py','leap_middle_clearance_probe.cmd','screen_middle_probe_current_20261106.py','build_middle_clearance_probe_20261106.py','screen_middle_probe_hand_20261106.py','prepare_middle_probe_review_20261106.py','verified_off_restore_context_20261105.py')
ALIAS_MAP={FOLD:'stage_7_horizontal_yaw',HOLD:'stage_8_horizontal_hold',RETURN:'stage_7_horizontal_yaw',RESTORED:'stage_8_horizontal_hold'}
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def validate_arrays(start,goals,phases,old,metadata,alias):
    lo,hi=limits();goals=np.asarray(goals);phases=np.asarray(phases)
    if goals.ndim!=2 or goals.shape[1]!=16 or not np.isfinite(goals).all() or np.any(goals!=np.rint(goals)) or np.any(goals<lo) or np.any(goals>hi):raise RuntimeError('Probe goal/native bound')
    if not np.array_equal(start,old['start']) or max(abs(np.diff(np.vstack([start,goals]),axis=0)).ravel())>2:raise RuntimeError('Probe start/2-tick bound')
    if list(dict.fromkeys(phases.tolist()))!=['stage_5_loaded_grasp','stage_6_grasp_hold',FOLD,HOLD,RETURN,RESTORED,'stage_9_loaded_open','stage_10_remove_hold']:raise RuntimeError('Probe phases changed')
    if not np.array_equal(goals[:927],old['goals'][:927]) or not np.array_equal(goals[-881:],old['goals'][927:]):raise RuntimeError('Physically reached grasp/open source changed')
    grip=old['goals'][926];fold=goals[phases==FOLD];back=goals[phases==RETURN];target=np.asarray(metadata['middle_probe_goal'])
    if not np.array_equal(fold[-1],target) or not np.array_equal(back[-1],grip) or not np.array_equal(np.vstack([fold,back])[:,FIXED],np.tile(grip[FIXED],(len(fold)+len(back),1))):raise RuntimeError('Thumb/index/ring changed during middle-only probe')
    if len(fold)!=48 or len(back)!=48 or not np.all(goals[phases==HOLD]==target) or sum(phases==HOLD)!=67 or sum(phases==RESTORED)!=20 or not np.all(goals[phases==RESTORED]==grip):raise RuntimeError('Probe travel/fixed hold changed')
    if metadata['evaluation_aliases']!=ALIAS_MAP:raise RuntimeError('Evaluation phase meanings changed')
    expected=np.asarray([ALIAS_MAP.get(str(p),str(p)) for p in phases])
    if not np.array_equal(alias['start'],start) or not np.array_equal(alias['goals'],goals) or not np.array_equal(alias['phases'],expected):raise RuntimeError('Evaluator aliases differ from real goals/phases')
    if int(alias['forward_end_sample'])!=927+48+67:raise RuntimeError('Probe observation boundary changed')
def validate_candidate():
    z=np.load(CANDIDATE);old=natural_candidate();p=json.loads((O/'proposal.json').read_bytes());a=np.load(ALIAS)
    teach=R/'manual_grasp_teach_20261106_183247_578628/result.json';t=json.loads(teach.read_bytes());expected=old['goals'][926].copy();expected[MIDDLE]+=np.rint(.5*(np.array(t['corrected_position_ticks'])[MIDDLE]-np.array(t['last_powered_position_ticks'])[MIDDLE])).astype(int)
    if p['candidate_sha256']!=sha(CANDIDATE) or p['evaluation_alias_sha256']!=sha(ALIAS) or p['source_natural_sha256']!=sha(R/'right_hand_rotation_20261030/natural_cradle_grasp/candidate.npz') or p['source_teaching_sha256']!=sha(teach) or p['source_script_sha256']!=sha(R/'build_middle_clearance_probe_20261106.py'):raise RuntimeError('Probe source hash changed')
    if p['source_curvature_fraction']!=.5 or not np.array_equal(p['middle_probe_goal'],expected):raise RuntimeError('Probe no longer matches half of the taught correction')
    validate_arrays(z['start'],z['goals'],z['phases'],old,p,a);return z
def hashes():
    files=[*(R/n for n in FILES),REQUEST,CANDIDATE,ALIAS,O/'proposal.json',R/'manual_grasp_teach_20261106_183247_578628/result.json',O/'loaded_hand_dynamics/review.json',*(O/f'free_front_{f}/review.json' for f in (-15,-10,-5))]
    return {**natural_hashes(),**{p.relative_to(R).as_posix():sha(p) for p in files}}
class MiddleProbePlan(NaturalCradleGraspPlan):
    PLAN_VERSION='supervised_middle_only_curvature_probe_return_open_v1_20261106'
    TITLE='중지 공간 확인: 빈손 준비 -> PLACE -> 기존 파지 -> 중지만 접기/5초 -> 복귀/OPEN -> REMOVE/OFF.'
    FOLDER_PREFIX='middle_clearance_probe';STATUS='middle_clearance_probe_observation_complete'
    COMPLETE_MESSAGE='중지 보정 관찰/복귀 완료. 큐브 제거 및 OFF/설정 복구를 기록했습니다. 완성된 하부 파지나 큐브 회전은 미검증입니다.'
    proof_hashes=staticmethod(hashes)
    def __init__(self,request):
        super().__init__(request)
        # Same bounded OFF centering as SingleForwardPlan: reuse only a source
        # within four ticks. The preload still uses fresh actual readings and
        # the original four-tick enable/health checks remain unchanged.
        if self.SOURCE.resolve()!=REQUEST.resolve() and REQUEST.exists():
            center=np.asarray(json.loads(REQUEST.read_bytes())['off_state']['position'],int)
            if np.max(abs(self.raw_off_ticks-center))<=4:self.center=center;self.review_center_reused=True
    def plan(self,raw):
        a=self.validate_start(raw);z=validate_candidate();rows=prefix_rows(self,a)+[(str(p),q.copy()) for p,q in zip(z['phases'],z['goals'])];values=validate_path(a,rows,ENTRY)
        if np.max(abs(values[:,:4]-a[:4]))>self.MAX_THUMB_TRAVEL or (len(rows)+40)*self.SAMPLE_PERIOD+20>self.ON_TIMEOUT:raise RuntimeError('Existing thumb travel/time limit exceeded')
        return rows
    def frozen_for_phase(self,phase):
        if phase in (FOLD,RETURN):return FIXED
        if phase in (HOLD,RESTORED):return np.arange(16)
        return super().frozen_for_phase(phase)
    def progress_for_phase(self,phase):
        if phase in (FOLD,RETURN):return MIDDLE
        if phase in (HOLD,RESTORED):return np.array([],int)
        return super().progress_for_phase(phase)
