"""Draft slow yaw from the retained RED-up pose; no hardware access.

The unchanged yaw targets repeat once at the same75ms cycle, giving150ms
between distinct yaw goals. No motion authorization or execution is provided.
"""
import hashlib,json
from pathlib import Path
import numpy as np
from calibrated_quarter_plan_20261028 import HorizontalQuarterPlan as BasePlan,LOADED_PATH as ORIGINAL,hashes as base_hashes
from calibrated_empty_ring_margin_20261028 import rows as prefix_rows
from horizontal_quarter_plan_20261027 import PHASE_MAP
from loaded_quarter_reentry_20261025 import validate_path
from new_pair_grasp_plan_20261027 import ENTRY

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261029/retained_red_yaw_candidate'
REQUEST=OUT/'request.json';PATH=OUT/'loaded_path.npz';PROOF=OUT/'review.json'
VERSION='retained_red_up_yaw150ms_candidate_v1_20261029'

def prepare():
    OUT.mkdir(parents=True,exist_ok=True)
    source=ROOT/'prefix638_off_readonly_20261029_001123_114252/result.json'
    r=json.loads(source.read_bytes())
    assert r['status']=='off_snapshots_saved' and r['servo_writes']==0 and r['torque_changes']==0 and r['maximum_drift_ticks']<=2
    request=dict(plan_version=VERSION,off_state=r['snapshots'][-1],source=source.relative_to(ROOT).as_posix(),
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),motor_ids_in_sim_order=__import__('hand_motor_mapping').SIM_TO_MOTOR_IDS,
        hardware_execution_allowed=False)
    REQUEST.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8')
    z=np.load(ORIGINAL);rows=[];phases=[];indices=[]
    for i,(q,p) in enumerate(zip(z['goals'],z['phases'])):
        for _ in range(2 if str(p)=='yaw' else 1):rows.append(q.copy());phases.append(str(p));indices.append(i)
    np.savez_compressed(PATH,start=z['start'],goals=np.asarray(rows),phases=np.asarray(phases),
        source_indices=np.asarray(indices),sample_period_s=.075,distinct_yaw_goal_period_s=.15)

def hashes():
    return {**base_hashes(),**{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in (Path(__file__),REQUEST,PATH)}}

def loaded_rows():
    z=np.load(PATH);old=np.load(ORIGINAL)
    expected=np.repeat(np.arange(len(old['goals'])),np.where(old['phases']=='yaw',2,1))
    np.testing.assert_array_equal(z['source_indices'],expected)
    np.testing.assert_array_equal(z['goals'],old['goals'][expected])
    np.testing.assert_array_equal(z['phases'],old['phases'][expected])
    np.testing.assert_array_equal(z['start'],old['start'])
    return [(PHASE_MAP[str(p)],q.copy()) for p,q in zip(z['phases'],z['goals'])]

class RetainedRedYawPlan(BasePlan):
    PLAN_VERSION=VERSION
    TITLE='DRAFT RED-UP YAW: EMPTY preparation -> PLACE RED up/GREEN wristward -> slow yaw -> OPEN -> REMOVE -> OFF.'
    FOLDER_PREFIX='retained_red_yaw';STATUS='retained_red_yaw_observation_complete'
    COMPLETE_MESSAGE='Retained-pose yaw observation complete. Actual cube rotation requires saved-image review.'
    proof_hashes=staticmethod(hashes)
    def __init__(self,request):
        super().__init__(request)
        self.raw_off_ticks=self.center.copy();self.review_center_reused=False
        if self.SOURCE.resolve()!=REQUEST.resolve() and REQUEST.is_file():
            center=np.asarray(json.loads(REQUEST.read_bytes())['off_state']['position'],dtype=int)
            if np.max(abs(self.raw_off_ticks-center))<=4:self.center=center;self.review_center_reused=True
    def plan(self,raw):
        a=self.validate_start(raw);rows=prefix_rows(self,a)+loaded_rows();values=validate_path(a,rows,ENTRY)
        if (len(rows)+40)*self.SAMPLE_PERIOD+20>self.ON_TIMEOUT:raise ValueError('Existing300s active-time bound exceeded')
        if np.max(abs(values[:,:4]-a[:4]))>self.MAX_THUMB_TRAVEL:raise ValueError('Existing thumb-travel bound exceeded')
        return rows

if __name__=='__main__':
    prepare();p=RetainedRedYawPlan(REQUEST);rows=p.plan(p.source_position())
    print(json.dumps(dict(status='draft_only_no_execution_entrypoint',request=str(REQUEST),loaded_path=str(PATH),
        total_goals=len(rows),nominal_motion_s=len(rows)*.075,active_limit_s=p.ON_TIMEOUT,
        no_angular_targets_changed=True,distinct_yaw_period_s=.15,review_file_present=PROOF.exists(),devices_opened=False)))
