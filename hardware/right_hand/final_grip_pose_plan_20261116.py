"""Empty-hand inspection of the saved TH/MF/RF model pose; no cube rotation."""
import hashlib
import json
from pathlib import Path
import numpy as np
from forward_hold_plan_20261104 import ForwardHoldPlan
from single_forward_plan_20261030 import RawBase
from four_reply_yaw_plan_20261029 import hashes as transport_hashes
from calibrated_empty_ring_margin_20261028 import rows as prefix_rows
from loaded_quarter_reentry_20261025 import limits
from final_pose_start_scope_20261116 import admission,healthy_off,validate_path
from new_pair_grasp_plan_20261027 import ENTRY

ROOT = Path(__file__).resolve().parent
OUT = ROOT/'right_hand_rotation_20261116/final_grip_pose'
REQUEST = OUT/'request.json'
PROOF = OUT/'review.json'
CANDIDATE = OUT/'candidate.npz'
MANIFEST = OUT/'manifest.json'
REFERENCE = ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/ring_thumb_MF_support_feedback_v2/force0.5/end_state.npz'
MOVE = 'final_grip_empty_move'
HOLD = 'final_grip_empty_settle'
FILES = ('final_grip_pose_plan_20261116.py', 'final_grip_pose_gate_20261116.py',
         'final_grip_pose_capture_20261116.py', 'leap_final_grip_pose_20261116.py',
         'leap_final_grip_pose.cmd', 'screen_final_grip_pose_current_20261116.py',
         'screen_final_grip_pose_full_20261116.py', 'empty_start_thumb_index_scope_20261113.py',
         'screen_empty_initial_scope_20261113.py', 'leap_forward_hold_observe_20261104.py',
         'verified_off_restore_context_20261105.py', 'forward_hold_plan_20261104.py',
         'single_forward_plan_20261030.py', 'replay_final_pose_guards_20261116.py',
         'final_pose_start_scope_20261116.py', 'screen_final_pose_recovery_20261116.py',
         'final_pose_temporary_support_20261116.py','interrupted_finger_scope_20261116.py',
         'final_grip_standard_read_20261116.py','final_grip_single_instance_20261116.py',
         'final_grip_bus_20261116.py')

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def candidate():
    m = json.loads(MANIFEST.read_bytes())
    if sha(CANDIDATE) != m['candidate_sha256'] or sha(REFERENCE) != m['reference_sha256']:
        raise RuntimeError('Final pose source changed; no writes')
    z = dict(np.load(CANDIDATE, allow_pickle=False))
    from hand_motor_mapping import SIM_TO_MOTOR_IDS, TICK
    target = np.empty(16, dtype=int)
    target[np.asarray(SIM_TO_MOTOR_IDS)-1] = np.rint(np.load(REFERENCE)['qpos'][:16]/TICK+2048).astype(int)
    offset = np.asarray(m.get('empty_clearance_offset_ticks', [0]*16))
    if (offset.shape != (16,) or not np.isfinite(offset).all()
            or np.any(offset != np.rint(offset)) or np.max(abs(offset)) > 4
            or np.count_nonzero(offset) > 1):
        raise RuntimeError('Unreviewed empty-pose clearance offset')
    target += offset.astype(int)
    lo, hi = limits()
    values = np.vstack([z['start'], z['goals']])
    if (z['goals'].ndim != 2 or z['goals'].shape[1] != 16
            or len(z['phases']) != len(z['goals']) or not np.isfinite(values).all()
            or np.any(values != np.rint(values)) or np.any(values < lo) or np.any(values > hi)
            or np.max(abs(np.diff(values, axis=0))) > 2
            or not np.array_equal(z['goals'][-1], target)
            or set(z['phases']) != {MOVE, HOLD}
            or not np.all(z['goals'][z['phases'] == HOLD] == target)):
        raise RuntimeError('Empty final pose range/rate/target contract rejected')
    return z

def hashes():
    return {**transport_hashes(), **{p.relative_to(ROOT).as_posix(): sha(p)
        for p in [*(ROOT/n for n in FILES), CANDIDATE, MANIFEST, REQUEST, REFERENCE,
                  OUT/'runtime_guard_review.json']}}

class FinalGripPosePlan(ForwardHoldPlan):
    PLAN_VERSION = 'empty_saved_final_TH_MF_RF_pose_hold_v1_20261116'
    TITLE = '빈손 최종 파지 모양 확인: 천천히 이동 -> 같은 목표 유지 -> SPACE 종료/OFF'
    FOLDER_PREFIX = 'final_grip_pose'
    STATUS = 'empty_final_grip_pose_observation_complete'
    COMPLETE_MESSAGE = '빈손 자세 확인 종료. 토크 OFF와 설정 복구를 기록했습니다.'
    ON_TIMEOUT = 500
    CAPTURE_PLACEMENT = True
    EMPTY_RELEASE_ONLY = True
    CAPTURE_BEFORE_PHASE = None
    CONTACT_RELEASE_BEFORE_PHASE = None
    MODEL_LIMITS_DESCRIPTION = 'unchanged native limits; EMPTY hand only; no cube placement'
    proof_hashes = staticmethod(hashes)
    validate_admission=staticmethod(admission)
    validate_off_state=staticmethod(healthy_off)

    def __init__(self, request):
        RawBase.__init__(self, request)
        self.raw_off_ticks = self.center.copy()
        self.review_center_reused = False

    def plan(self, raw):
        a = self.validate_start(raw)
        z = candidate()
        prefix=empty_prefix(self,a)
        rows = prefix + [(str(p), q.copy()) for p, q in zip(z['phases'], z['goals'])]
        values = validate_path(a, rows, ENTRY)
        if (np.max(abs(values[:, :4]-a[:4])) > self.MAX_THUMB_TRAVEL
                or (len(rows)+40)*self.SAMPLE_PERIOD+20 > self.ON_TIMEOUT):
            raise RuntimeError('Existing travel or maximum ON time exceeded')
        return rows

    def frozen_for_phase(self, phase):
        if phase == ENTRY:
            return np.asarray([i for i in range(16) if i not in (3,4,6,7,11,15)])
        if phase == MOVE:
            return np.array([], dtype=int)
        if phase in (HOLD, 'stage_2_fixed_hold', 'final_grip_user_hold'):
            return np.arange(16)
        return super().frozen_for_phase(phase)

    def progress_for_phase(self, phase):
        if phase == ENTRY:
            return np.asarray([3,4,6,7,11,15])
        if phase == MOVE:
            return np.arange(16)
        if phase in (HOLD, 'stage_2_fixed_hold', 'final_grip_user_hold'):
            return np.array([], dtype=int)
        return super().progress_for_phase(phase)

def empty_prefix(plan,a):
        from copy import copy
        from calibrated_empty_ring_margin_20261028 import interpolate
        a=plan.validate_start(a);escaped=a.copy()
        needs_escape=a[4]>2400 and a[8]>2900 and a[6]<1900 and a[10]<2400
        if needs_escape:escaped[4]-=64
        escape=[] if not needs_escape else [(ENTRY,q) for q in interpolate(a,escaped)]
        lo,_=limits();inside=escaped.copy()
        inside[[3,6,7,11,15]]=np.maximum(inside[[3,6,7,11,15]],np.ceil(lo[[3,6,7,11,15]]).astype(int))
        if a[3]<lo[3]:inside[3]=int(np.ceil(lo[3]))+8
        for j in (6,7,11,15):
            if a[j]<lo[j]:inside[j]=int(np.ceil(lo[j]))+64
        proxy=copy(plan);proxy.center=inside.copy()
        lead=[] if np.array_equal(escaped,inside) else [(ENTRY,q) for q in interpolate(escaped,inside)]
        prefix = escape+lead+prefix_rows(proxy, inside)
        if not np.array_equal(prefix[-1][1], candidate()['start']):
            raise RuntimeError('Empty approach does not join screened open pose')
        validate_path(a,prefix,ENTRY)
        return prefix
