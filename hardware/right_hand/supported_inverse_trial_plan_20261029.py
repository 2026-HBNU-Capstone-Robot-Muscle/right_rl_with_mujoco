"""Supervised cube observation of the completed supported EMPTY trajectory."""
import json
import numpy as np
from supported_inverse_empty_plan_20261029 import (SupportedInverseEmptyPlan,ROOT,LOADED,HAND_PATH,new_rows,sha,hashes as empty_hashes)
from calibrated_quarter_plan_20261028 import HorizontalQuarterPlan as RawBase
from hand_motor_mapping import SIM_TO_MOTOR_IDS
OUT=ROOT/'right_hand_rotation_20261029/supported_inverse_trial'
REQUEST=OUT/'request.json';PROOF=OUT/'review.json'
ACTUAL=ROOT/'supported_inverse_empty_20261029_225256_285948/result.json'
ACTUAL_REVIEW=ACTUAL.parent/'offline_review/review.json'
SEED=ROOT/'right_hand_rotation_20261029/gapped_inverse_cube/cube_seed.json'
MODEL_SOURCES=[ROOT/'right_hand_rotation_20261029/supported_inverse_search/support_grid'/name/'review.json' for name in ('full9','full9_front_-15','full9_front_-5')]
FILES=('supported_inverse_trial_plan_20261029.py','supported_inverse_trial_gate_20261029.py','supported_inverse_trial_capture_20261029.py',
    'screen_supported_inverse_trial_20261029.py','leap_supported_inverse_trial_20261029.py','leap_supported_inverse_trial.cmd',
    'screen_material_checked_reverse_cube_20261029.py','cube_contact_material_policy_20261029.py')
def hashes():
    return {**empty_hashes(),**{p.relative_to(ROOT).as_posix():sha(p) for p in [*(ROOT/f for f in FILES),REQUEST,ACTUAL,ACTUAL_REVIEW,SEED,*MODEL_SOURCES]}}
class SupportedInverseTrialPlan(SupportedInverseEmptyPlan):
    PLAN_VERSION='supported_inverse_same_completed_EMPTY_route_cube_observation_v1_20261029'
    TITLE='SUPPORTED INVERSE OBSERVATION: EMPTY prepare -> PLACE -> close/support/turn -> OPEN -> REMOVE -> OFF.'
    FOLDER_PREFIX='supported_inverse_trial';STATUS='supported_inverse_observation_complete'
    COMPLETE_MESSAGE='새 역회전 경로 관찰 완료. 큐브 제거, OFF와 설정 복구를 기록했습니다. 회전각은 저장 영상으로 확인합니다.'
    CAPTURE_PLACEMENT=True;CAPTURE_BEFORE_PHASE='stage_5_loaded_grasp';CONTACT_RELEASE_BEFORE_PHASE=None;EMPTY_RELEASE_ONLY=False
    proof_hashes=staticmethod(hashes)
    def __init__(self,request):
        RawBase.__init__(self,request);self.raw_off_ticks=self.center.copy();self.review_center_reused=False
        if self.SOURCE.resolve()!=REQUEST.resolve() and REQUEST.exists():
            center=np.asarray(json.loads(REQUEST.read_bytes())['off_state']['position'],int)
            if np.max(abs(self.raw_off_ticks-center))<=4:self.center=center;self.review_center_reused=True
def prepare():
    source=ROOT/'fast_off_readonly_20261029_225900_662649/result.json';r=json.loads(source.read_bytes())
    assert r['status']=='off_snapshots_saved' and len(r['snapshots'])==3 and r['maximum_drift_ticks']<=2
    assert r['servo_writes']==r['torque_changes']==0
    OUT.mkdir(exist_ok=True)
    if REQUEST.exists():raise RuntimeError('Refuse to overwrite prepared cube request')
    REQUEST.write_text(json.dumps(dict(plan_version=SupportedInverseTrialPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
        off_state=r['snapshots'][-1],source=source.relative_to(ROOT).as_posix(),source_sha256=sha(source),hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
    p=SupportedInverseTrialPlan(REQUEST);rows=p.plan(p.source_position())
    print(json.dumps(dict(goals=len(rows),nominal_motion_s=len(rows)*p.SAMPLE_PERIOD,devices_opened=False)))
if __name__=='__main__':prepare()
