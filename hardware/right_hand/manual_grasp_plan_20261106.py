"""Replay the executed grasp prefix; no OPEN or motion after torque-OFF teaching."""
import hashlib,json
from pathlib import Path
import numpy as np
from natural_cradle_grasp_plan_20261030 import NaturalCradleGraspPlan,validate_candidate as original_candidate,hashes as original_hashes
R=Path(__file__).resolve().parent;OUT=R/'right_hand_rotation_20261106/manual_grasp';REQUEST=OUT/'request.json';PROOF=OUT/'review.json';CANDIDATE=OUT/'candidate.npz';SOURCE_REVIEW=OUT/'source_review.json'
GRASP='stage_5_loaded_grasp';HOLD='stage_6_grasp_hold';OPEN='stage_9_loaded_open'
FILES=('manual_grasp_plan_20261106.py','manual_grasp_gate_20261106.py','manual_grasp_capture_20261106.py','manual_grasp_off_teach_20261106.py','leap_manual_grasp_teach_20261106.py','leap_manual_grasp_teach.cmd','screen_manual_grasp_current_20261106.py','verified_off_restore_context_20261105.py')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def prefix(source):
    phases=np.asarray(source['phases']);goals=np.asarray(source['goals']);holds=np.flatnonzero(phases==HOLD)
    if not len(holds):raise ValueError('Executed grasp hold missing')
    cut=int(holds[-1])+1;q=goals[:cut];p=phases[:cut]
    if list(dict.fromkeys(p.tolist()))!=[GRASP,HOLD] or len(holds)!=67 or not np.array_equal(q[-67:],np.tile(q[-1],(67,1))):raise ValueError('Grasp/5s hold differs from source')
    if np.max(abs(np.diff(np.vstack([source['start'],q]),axis=0)))>2:raise ValueError('Goal step exceeded')
    return q,p
def validate_candidate():
    original=original_candidate();expected,phases=prefix(original);z=np.load(CANDIDATE);r=json.loads(SOURCE_REVIEW.read_bytes());physical=R/r['source_physical_result'];executed=json.loads(physical.read_bytes())
    if sha(physical)!=r['source_physical_result_sha256'] or not executed['all_torque_off_verified'] or not executed['settings_restored'] or executed['status']!='natural_cradle_grasp_observation_complete':raise RuntimeError('Executed source changed or incomplete')
    if not np.array_equal(z['start'],original['start']) or not np.array_equal(z['goals'],expected) or not np.array_equal(z['phases'],phases) or sha(CANDIDATE)!=r['candidate_sha256']:raise RuntimeError('Only the executed927-goal grasp prefix is permitted')
    if not np.array_equal(executed['grasp_goal_ticks'],z['goals'][-1]):raise RuntimeError('Source physical grip target differs')
    return z
def hashes():
    paths=[*(R/n for n in FILES),REQUEST,CANDIDATE,SOURCE_REVIEW]
    return {**original_hashes(),**{p.relative_to(R).as_posix():sha(p) for p in paths}}
class ManualGraspPlan(NaturalCradleGraspPlan):
    PLAN_VERSION='executed_grasp_support_OFF_then_manual_joint_readonly_v1_20261106'
    TITLE='손가락 보정: 빈손 준비 -> PLACE -> 이전 파지 -> 받치고SPACE OFF -> 손가락 보정/읽기만 저장.'
    FOLDER_PREFIX='manual_grasp_teach_motion';STATUS='manual_grasp_reached_torque_off'
    COMPLETE_MESSAGE='파지 자세에서 토크OFF/설정 복구 완료. 다음 읽기 전용 단계에서 손가락 각도를 보정합니다.'
    proof_hashes=staticmethod(hashes)
    def plan(self,raw):
        z=validate_candidate();complete=super().plan(raw);cut=next(i for i,(p,q) in enumerate(complete) if p==OPEN);result=complete[:cut];loaded=[(p,q) for p,q in result if p in (GRASP,HOLD)]
        if len(loaded)!=len(z['goals']) or any(p!=phase or not np.array_equal(q,goal) for (p,q),phase,goal in zip(loaded,z['phases'],z['goals'])):raise RuntimeError('Executed grasp prefix not retained exactly')
        if any(p in ('stage_9_loaded_open','stage_10_remove_hold') for p,q in result):raise RuntimeError('Automatic OPEN is forbidden in teaching mode')
        return result
