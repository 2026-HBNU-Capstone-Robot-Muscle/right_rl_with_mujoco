"""New supported inverse trajectory, operator-started and EMPTY throughout."""
import hashlib,json
from pathlib import Path
import numpy as np
from gapped_inverse_plan_20261029 import GappedInversePlan,extra_rows,hashes as inherited_hashes
from calibrated_quarter_plan_20261028 import HorizontalQuarterPlan as RawBase
from calibrated_empty_ring_margin_20261028 import rows as prefix_rows
from loaded_quarter_reentry_20261025 import validate_path
from new_pair_grasp_plan_20261027 import ENTRY,OPEN_PATH
from hand_motor_mapping import SIM_TO_MOTOR_IDS

ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261029/supported_inverse_empty'
REQUEST=OUT/'request.json';PROOF=OUT/'review.json';LOADED=OUT/'candidate.npz';HAND_PATH=OUT/'whole_hand_path.npz'
FILES=('supported_inverse_empty_plan_20261029.py','supported_inverse_empty_gate_20261029.py',
    'screen_supported_inverse_empty_20261029.py','screen_supported_inverse_geometry_20261029.py',
    'screen_supported_inverse_hand_20261029.py','leap_supported_inverse_empty_20261029.py','leap_supported_inverse_empty.cmd',
    'screen_loaded_yaw_static_20261027.py','screen_pitch_handoff_loaded_20261028.py',
    'leap_yaw_load_empty_20261029.py')
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def hashes():
    paths=[*(ROOT/f for f in FILES),REQUEST,LOADED,HAND_PATH,OUT/'whole_geometry/review.json',OUT/'loaded_hand_dynamics/review.json']
    return {**inherited_hashes(),**{p.relative_to(ROOT).as_posix():sha(p) for p in paths}}
def new_rows():
    z=np.load(LOADED);return extra_rows()+[(str(p),q.copy()) for p,q in zip(z['phases'],z['goals'])]

class SupportedInverseEmptyPlan(GappedInversePlan):
    PLAN_VERSION='EMPTY_supported_inverse_contact_20261029_v1'
    TITLE='EMPTY SUPPORTED INVERSE CHECK: preparation -> index support/new thumb-middle route -> OPEN -> OFF. No cube.'
    FOLDER_PREFIX='supported_inverse_empty';STATUS='supported_inverse_empty_path_complete'
    COMPLETE_MESSAGE='새 역회전 빈손 경로 완료. OPEN, OFF와 설정 복구를 기록했습니다. 큐브 회전은 이번에 시험하지 않았습니다.'
    CAPTURE_PLACEMENT=True;CAPTURE_BEFORE_PHASE=None;CONTACT_RELEASE_BEFORE_PHASE=None;EMPTY_RELEASE_ONLY=True
    proof_hashes=staticmethod(hashes)
    def __init__(self,request):
        RawBase.__init__(self,request);self.raw_off_ticks=self.center.copy();self.review_center_reused=False
        if self.SOURCE.resolve()!=REQUEST.resolve() and REQUEST.exists():
            center=np.asarray(json.loads(REQUEST.read_bytes())['off_state']['position'],int)
            if np.max(abs(self.raw_off_ticks-center))<=4:self.center=center;self.review_center_reused=True
    def plan(self,raw):
        start=self.validate_start(raw);rows=prefix_rows(self,start)+new_rows();values=validate_path(start,rows,ENTRY)
        if np.max(abs(values[:,:4]-start[:4]))>self.MAX_THUMB_TRAVEL or (len(rows)+40)*self.SAMPLE_PERIOD+20>self.ON_TIMEOUT:
            raise RuntimeError('Existing thumb travel/time bound exceeded')
        return rows

def prepare():
    source=ROOT/'fast_off_readonly_20261029_222351_976318/result.json';r=json.loads(source.read_bytes())
    assert r['status']=='off_snapshots_saved' and len(r['snapshots'])==3 and r['maximum_drift_ticks']<=2
    assert r['servo_writes']==r['torque_changes']==0
    OUT.mkdir(exist_ok=True)
    if REQUEST.exists() or LOADED.exists() or HAND_PATH.exists():raise RuntimeError('Refuse to overwrite frozen preparation')
    candidate=ROOT/'right_hand_rotation_20261029/supported_inverse_search/support_grid/candidate_001.npz'
    LOADED.write_bytes(candidate.read_bytes())
    REQUEST.write_text(json.dumps(dict(plan_version=SupportedInverseEmptyPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
        off_state=r['snapshots'][-1],source=source.relative_to(ROOT).as_posix(),source_sha256=sha(source),hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
    rows=new_rows();np.savez_compressed(HAND_PATH,start=np.load(OPEN_PATH)['open_goal'],goals=np.array([q for _,q in rows]),phases=np.array([p for p,_ in rows]))
    p=SupportedInverseEmptyPlan(REQUEST);allrows=p.plan(p.source_position())
    print(json.dumps(dict(whole_goals=len(allrows),nominal_seconds=len(allrows)*p.SAMPLE_PERIOD,new_hand_goals=len(rows),cube_stays_OUT=True,devices_opened=False)))
if __name__=='__main__':prepare()
