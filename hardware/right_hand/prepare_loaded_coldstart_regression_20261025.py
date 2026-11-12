import json
from pathlib import Path
from loaded_quarter_trial_plan_20261025 import ROOT,LoadedQuarterTrialPlan
out=ROOT/'right_hand_rotation_20261024/loaded_quarter_trial_20261025/fixture_v5_regression_0915';out.mkdir(exist_ok=True)
r=json.loads((ROOT/'completed_empty_readonly_20261025_091531_043679/request.json').read_bytes());r['plan_version']=LoadedQuarterTrialPlan.PLAN_VERSION
(out/'request.json').write_text(json.dumps(r,indent=2),encoding='utf-8')
p=LoadedQuarterTrialPlan(out/'request.json');print('REGRESSION_GOALS',len(p.plan(p.source_position())))
