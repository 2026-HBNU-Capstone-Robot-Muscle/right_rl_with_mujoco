import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
path=ROOT/'support_first_trial_20261112_075306_271459/result.json'
r=json.loads(path.read_bytes())
print(json.dumps({k:r.get(k) for k in ['status','error','all_torque_off_verified','settings_restored','physical_transfer_success','physical_two_turn_success']},ensure_ascii=False))
print('phase_report',r.get('support_first_phases_seen'))
from support_first_trial_plan_20261112 import SupportFirstTrialPlan
from tip_first_trial_plan_20261112 import TipFirstTrialPlan
request=ROOT/'tip_first_trial_20261112_173936_071900/fresh_off_request.json'
def prefix(cls):
    p=cls(request);rows=p.plan(p.source_position())
    return [(phase,q.tolist()) for phase,q in rows if phase.startswith(('stage_0','stage_1_','stage_3_empty','stage_4_place'))]
try:
    a,b=prefix(SupportFirstTrialPlan),prefix(TipFirstTrialPlan)
    print('SAME_CURRENT_EMPTY_PREFIX',a==b,'GOALS',len(a),len(b))
except Exception as exc:print(type(exc).__name__,str(exc)[:400])
