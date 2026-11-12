"""Supervised forward/inverse composition; keep the Cube grasped at the join."""
import json,hashlib
from pathlib import Path
import numpy as np
from four_reply_yaw_plan_20261029 import FourReplyYawPlan as Base,hashes as base_hashes
from calibrated_quarter_plan_20261028 import HorizontalQuarterPlan as RawBase
from calibrated_empty_ring_margin_20261028 import rows as prefix_rows
from loaded_quarter_reentry_20261025 import validate_path
from new_pair_grasp_plan_20261027 import ENTRY,OPEN_PATH
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from gapped_inverse_plan_20261029 import hashes as inverse_hashes
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261030/yaw_roundtrip'
REQUEST=OUT/'request.json';PROOF=OUT/'review.json';LOADED=OUT/'candidate.npz';HAND_PATH=OUT/'whole_hand_path.npz'
MODEL_SOURCES=[OUT/f'full_loaded_seated_front_{f}/review.json' for f in (-15,-10,-5)]
FILES=('yaw_roundtrip_plan_20261030.py','yaw_roundtrip_gate_20261030.py','yaw_roundtrip_capture_20261030.py',
    'screen_yaw_roundtrip_20261030.py','screen_yaw_roundtrip_hand_20261030.py','screen_yaw_roundtrip_model_20261030.py',
    'leap_yaw_roundtrip_trial_20261030.py','leap_yaw_roundtrip_trial.cmd','cube_contact_material_policy_20261029.py')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def hashes():
    paths=[*(ROOT/n for n in FILES),REQUEST,LOADED,HAND_PATH,OUT/'proposal.json',OUT/'loaded_hand_dynamics/review.json',*MODEL_SOURCES]
    return {**base_hashes(),**inverse_hashes(),**{p.relative_to(ROOT).as_posix():sha(p) for p in paths}}
def loaded_rows():
    z=np.load(LOADED);return [(str(p),q.copy()) for p,q in zip(z['phases'],z['goals'])]
class YawRoundtripPlan(Base):
    PLAN_VERSION='continuous_forward_inverse_observation_v1_20261030'
    TITLE='CONTINUOUS YAW: EMPTY -> PLACE GREEN up/ORANGE wristward -> forward638 -> inverse638 -> OPEN -> REMOVE/OFF.'
    FOLDER_PREFIX='yaw_roundtrip_trial';STATUS='yaw_roundtrip_observation_complete'
    COMPLETE_MESSAGE='연속 정회전/역회전 관찰 완료. 큐브 제거와 OFF/복구를 기록했습니다. 각도는 영상에서 확인합니다.'
    proof_hashes=staticmethod(hashes)
    CAPTURE_PLACEMENT=True;CAPTURE_BEFORE_PHASE='stage_5_loaded_grasp';CONTACT_RELEASE_BEFORE_PHASE=None;EMPTY_RELEASE_ONLY=False
    def __init__(self,request):
        RawBase.__init__(self,request);self.raw_off_ticks=self.center.copy();self.review_center_reused=False
        if self.SOURCE.resolve()!=REQUEST.resolve() and REQUEST.exists():
            ref=np.asarray(json.loads(REQUEST.read_bytes())['off_state']['position'],int)
            if np.max(abs(self.raw_off_ticks-ref))<=4:self.center=ref;self.review_center_reused=True
    def plan(self,raw):
        start=self.validate_start(raw);rows=prefix_rows(self,start)+loaded_rows();values=validate_path(start,rows,ENTRY)
        if np.max(abs(values[:,:4]-start[:4]))>self.MAX_THUMB_TRAVEL or (len(rows)+40)*self.SAMPLE_PERIOD+20>self.ON_TIMEOUT:
            raise RuntimeError('Composed route exceeds unchanged travel/time bounds')
        return rows
def prepare():
    source=ROOT/'fast_off_readonly_20261030_001016_621895/result.json';r=json.loads(source.read_bytes())
    assert r['status']=='off_snapshots_saved' and len(r['snapshots'])==3 and r['maximum_drift_ticks']<=2 and r['servo_writes']==r['torque_changes']==0
    if REQUEST.exists() or HAND_PATH.exists():raise RuntimeError('Refuse to overwrite frozen request/path')
    REQUEST.write_text(json.dumps(dict(plan_version=YawRoundtripPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],
        source=source.relative_to(ROOT).as_posix(),source_sha256=sha(source),hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
    z=np.load(LOADED);np.savez_compressed(HAND_PATH,start=z['start'],goals=z['goals'],phases=z['phases'])
    p=YawRoundtripPlan(REQUEST);rows=p.plan(p.source_position());print(json.dumps(dict(whole_goals=len(rows),loaded_goals=len(z['goals']),
        nominal_motion_seconds=len(rows)*p.SAMPLE_PERIOD,existing_motion_limit=p.ON_TIMEOUT,devices_opened=False)))
if __name__=='__main__':prepare()
