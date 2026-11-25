"""Draft continuous pitch -> handoff -> slower yaw, with one cube placement.

Only a source-bound, separately reviewed user-started operator may execute it.
Existing primitives, native ranges, drive profiles and health stops are intact.
"""
import hashlib,json
from pathlib import Path
import numpy as np
from calibrated_quarter_plan_20261028 import HorizontalQuarterPlan as BasePlan
from pitch_handoff_plan_20261028 import (prefix_rows,pitch_start,loaded_rows as pitch_rows,
    hashes as pitch_hashes,PITCH_PHASE,PITCH_HOLD,TRANSFER_PHASE,TRANSFER_HOLD)
from retained_red_yaw_plan_20261029 import loaded_rows as yaw_rows,PATH as YAW_PATH
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from loaded_quarter_reentry_20261025 import validate_path
from new_pair_grasp_plan_20261027 import ENTRY

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261029/pitch_then_slow_yaw_v1'
REQUEST=OUT/'request.json';PROOF=OUT/'review.json';LOADED_PATH=OUT/'loaded_path.npz'
VERSION='recorded_pitch_handoff_then_yaw150ms_v1_20261029'
FILES=('pitch_slow_yaw_plan_20261029.py','pitch_slow_yaw_capture_20261029.py',
    'pitch_slow_yaw_gate_20261029.py','leap_pitch_slow_yaw_20261029.py','leap_pitch_slow_yaw.cmd',
    'screen_pitch_slow_yaw_empty_20261029.py','screen_pitch_slow_yaw_hand_20261029.py',
    'screen_pitch_slow_yaw_review_20261029.py','retained_red_yaw_plan_20261029.py')

def expected_loaded_rows():
    a,b=pitch_rows(),yaw_rows()
    np.testing.assert_array_equal(a[-1][1],np.load(YAW_PATH)['start'])
    return a+b

def loaded_rows():
    z=np.load(LOADED_PATH);expected=expected_loaded_rows()
    np.testing.assert_array_equal(z['start'],pitch_start())
    np.testing.assert_array_equal(z['goals'],np.asarray([q for p,q in expected]))
    np.testing.assert_array_equal(z['phases'],np.asarray([p for p,q in expected]))
    return expected

def hashes():
    paths=[ROOT/f for f in FILES]+[REQUEST,LOADED_PATH,YAW_PATH,
        OUT/'loaded_hand_dynamics/review.json',OUT/'loaded_cube_evidence.json']
    return {**pitch_hashes(),**{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}}

class PitchSlowYawPlan(BasePlan):
    PLAN_VERSION=VERSION
    TITLE='CONTINUOUS TWO-AXIS EXPERIMENT: EMPTY -> PLACE -> pitch -> transfer -> SLOW yaw -> OPEN -> REMOVE -> OFF.'
    FOLDER_PREFIX='pitch_slow_yaw';STATUS='pitch_then_slow_yaw_observation_complete'
    COMPLETE_MESSAGE='Continuous pitch and slower yaw observation ended; cube removed; torque OFF/settings restored. Review actual faces before claiming success.'
    MAX_THUMB_TRAVEL=1568
    # This separate combined macro needs more wall time than a single primitive.
    # It preserves every motor/current/voltage/temperature/tracking criterion.
    # Its480s bound is checked against the complete queue before any enabling.
    # The previous pitch/yaw programs retain their300s bounds.
    ON_TIMEOUT=480
    CAPTURE_BEFORE_PHASE=PITCH_PHASE
    proof_hashes=staticmethod(hashes)
    def __init__(self,request):
        super().__init__(request)
        self.raw_off_ticks=self.center.copy();self.review_center_reused=False
        if self.SOURCE.resolve()!=REQUEST.resolve() and REQUEST.exists():
            center=np.asarray(json.loads(REQUEST.read_bytes())['off_state']['position'],dtype=int)
            if np.max(abs(self.raw_off_ticks-center))<=4:self.center=center;self.review_center_reused=True
    def plan(self,raw):
        a=self.validate_start(raw);rows=prefix_rows(self,a)+loaded_rows();values=validate_path(a,rows,ENTRY)
        if np.max(abs(values[:,:4]-a[:4]))>self.MAX_THUMB_TRAVEL:raise ValueError('Existing two-setup aggregate travel bound exceeded')
        if (len(rows)+40)*self.SAMPLE_PERIOD+20>self.ON_TIMEOUT:raise ValueError('Complete combined queue exceeds480s bound')
        return rows
    def frozen_for_phase(self,phase):
        if phase in (PITCH_PHASE,TRANSFER_PHASE):return np.array([],dtype=int)
        if phase in (PITCH_HOLD,TRANSFER_HOLD):return np.arange(16)
        return super().frozen_for_phase(phase)
    def progress_for_phase(self,phase):
        if phase in (PITCH_PHASE,TRANSFER_PHASE):return np.arange(16)
        if phase in (PITCH_HOLD,TRANSFER_HOLD):return np.array([],dtype=int)
        return super().progress_for_phase(phase)

def prepare():
    OUT.mkdir(parents=True,exist_ok=True)
    source=ROOT/'prefix638_off_readonly_20261029_001123_114252/result.json'
    r=json.loads(source.read_bytes())
    assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']<=2 and r['servo_writes']==0 and r['torque_changes']==0
    request=dict(plan_version=VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],
        source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),hardware_execution_allowed=False)
    REQUEST.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8')
    rows=expected_loaded_rows()
    np.savez_compressed(LOADED_PATH,start=pitch_start(),goals=np.asarray([q for p,q in rows]),phases=np.asarray([p for p,q in rows]),sample_period_s=.075)
    p=PitchSlowYawPlan(REQUEST);all_rows=p.plan(p.source_position())
    print(json.dumps(dict(status='draft_not_ready_for_execution',request=str(REQUEST),total_goals=len(all_rows),
        loaded_goals=len(rows),nominal_motion_s=len(all_rows)*.075,active_time_limit_s=p.ON_TIMEOUT,
        old_program_time_limits_unchanged=True,goal_step_ticks=2,current_ma=450,devices_opened=False)))

if __name__=='__main__':prepare()
