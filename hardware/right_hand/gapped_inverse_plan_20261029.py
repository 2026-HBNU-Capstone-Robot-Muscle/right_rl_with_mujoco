"""Operator-only inverse observation: EMPTY preposition, PLACE, close, turn, OPEN.

The task is an experimental roughly 30-degree inverse response, not a 90-degree
success claim. Motor settings and hardware health policies are inherited intact.
"""
import hashlib,json
from pathlib import Path
import numpy as np
from four_reply_yaw_plan_20261029 import FourReplyYawPlan as Base,hashes as base_hashes
from calibrated_quarter_plan_20261028 import HorizontalQuarterPlan as RawBase
from calibrated_empty_ring_margin_20261028 import rows as prefix_rows
from loaded_quarter_reentry_20261025 import validate_path
from new_pair_grasp_plan_20261027 import ENTRY,OPEN_PATH
from hand_motor_mapping import SIM_TO_MOTOR_IDS

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261029/gapped_inverse_cube'
REQUEST=OUT/'operator_request.json';PROOF=OUT/'operator_review.json'
LOADED=OUT/'candidate.npz';SEED=OUT/'cube_seed.json'
HAND_PATH=OUT/'whole_hand_path.npz';GEOMETRY_PATH=OUT/'additional_geometry.npz'
READY='stage_4_inverse_empty_ready';GAP='stage_4_inverse_empty_gap';GAP_HOLD='stage_4_inverse_empty_gap_hold'
FILES=('gapped_inverse_plan_20261029.py','gapped_inverse_gate_20261029.py',
    'gapped_inverse_capture_20261029.py','leap_gapped_inverse_trial_20261029.py',
    'leap_gapped_inverse_trial.cmd','screen_gapped_inverse_20261029.py',
    'screen_gapped_inverse_hand_20261029.py','screen_material_checked_reverse_cube_20261029.py',
    'cube_contact_material_policy_20261029.py','screen_reverse_loaded_cube_20261029.py')
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def hashes():
    paths=[*(ROOT/n for n in FILES),REQUEST,LOADED,SEED,HAND_PATH,GEOMETRY_PATH,
        OUT/'additional_geometry/review.json',OUT/'loaded_hand_dynamics/review.json',
        OUT/'request.json',OUT/'inverse_material_review.json',
        *(OUT/f'material_checked_front_{front}/review.json' for front in (-15,-10,-5))]
    return {**base_hashes(),**{p.relative_to(ROOT).as_posix():sha(p) for p in paths}}
def placement_goal():return np.load(LOADED)['start'].astype(int)
def extra_rows():
    z=np.load(LOADED)
    return ([(READY,q.copy()) for q in z['empty_preposition']]+
        [(GAP,q.copy()) for q in z['empty_gap_open']]+
        [(GAP_HOLD,z['start'].copy()) for _ in range(40)])
def loaded_rows():
    z=np.load(LOADED);return [(str(p),q.copy()) for p,q in zip(z['phases'],z['goals'])]

class GappedInversePlan(Base):
    PLAN_VERSION='gapped_prepositioned_inverse_observation_v1_20261029'
    TITLE='INVERSE OBSERVATION: EMPTY ready/gap -> PLACE WHITE wristward -> close -> reverse638 -> OPEN -> REMOVE/OFF.'
    FOLDER_PREFIX='gapped_inverse_trial';STATUS='gapped_inverse_observation_complete'
    COMPLETE_MESSAGE='역회전 관찰 완료. 큐브 제거, OFF와 설정 복구를 기록했습니다. 실제 각도는 영상에서 확인합니다.'
    CAPTURE_PLACEMENT=True;CAPTURE_BEFORE_PHASE='stage_5_loaded_grasp'
    CONTACT_RELEASE_BEFORE_PHASE=None;EMPTY_RELEASE_ONLY=False
    proof_hashes=staticmethod(hashes)
    def __init__(self,request):
        RawBase.__init__(self,request);self.raw_off_ticks=self.center.copy();self.review_center_reused=False
        if self.SOURCE.resolve()!=REQUEST.resolve() and REQUEST.exists():
            center=np.asarray(json.loads(REQUEST.read_bytes())['off_state']['position'],int)
            if np.max(abs(self.raw_off_ticks-center))<=4:self.center=center;self.review_center_reused=True
    def plan(self,raw):
        start=self.validate_start(raw)
        rows=prefix_rows(self,start)+extra_rows()+loaded_rows()
        values=validate_path(start,rows,ENTRY)
        if np.max(abs(values[:,:4]-start[:4]))>self.MAX_THUMB_TRAVEL or (len(rows)+40)*self.SAMPLE_PERIOD+20>self.ON_TIMEOUT:
            raise RuntimeError('Inverse trial exceeds existing travel/time bounds')
        return rows
    def frozen_for_phase(self,phase):
        if phase==READY:return np.array([4,5,6,7,12,13,14,15])
        if phase==GAP:return np.array([i for i in range(16) if i not in (0,1,10)])
        if phase==GAP_HOLD:return np.arange(16)
        return super().frozen_for_phase(phase)
    def progress_for_phase(self,phase):
        if phase==READY:return np.array([0,1,2,3,8,9,10,11])
        if phase==GAP:return np.array([0,1,10])
        if phase==GAP_HOLD:return np.array([],int)
        return super().progress_for_phase(phase)

def prepare():
    source=ROOT/'fast_off_readonly_20261029_180936_924857/result.json'
    r=json.loads(source.read_bytes())
    assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']<=2 and r['servo_writes']==0 and r['torque_changes']==0
    if REQUEST.exists():raise RuntimeError('Refuse to overwrite prepared request')
    REQUEST.write_text(json.dumps(dict(plan_version=GappedInversePlan.PLAN_VERSION,
        motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],
        source=source.relative_to(ROOT).as_posix(),source_sha256=sha(source),
        hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
    rows=extra_rows()+loaded_rows();opened=np.load(OPEN_PATH)['open_goal']
    np.savez_compressed(HAND_PATH,start=opened,goals=np.array([q for _,q in rows]),
        phases=np.array([p for p,_ in rows]))
    old=np.load(ROOT/'right_hand_rotation_20261029/reverse_yaw_empty/reverse_cycle.npz')
    covered={tuple(q) for q in np.vstack([old['start'],old['goals']])}
    new=sorted({tuple(q) for _,q in rows if tuple(q) not in covered})
    assert new
    np.savez_compressed(GEOMETRY_PATH,start=placement_goal(),goals=np.asarray(new,int))
    p=GappedInversePlan(REQUEST)
    print(json.dumps(dict(total_goals=len(p.plan(p.source_position())),new_geometry_poses=len(new),
        extra_empty_goals=len(extra_rows()),loaded_goals=len(loaded_rows()),devices_opened=False)))

if __name__=='__main__':prepare()
