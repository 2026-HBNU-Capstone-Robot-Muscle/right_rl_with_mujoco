import json
from pathlib import Path
from loaded_quarter_trial_plan_20261025 import ROOT,LoadedQuarterTrialPlan,validate_experiment_basis
from loaded_quarter_trial_gate_20261025 import validate_proof
folder=ROOT/'right_hand_rotation_20261024/loaded_quarter_trial_20261025/fixture_v3'
p=LoadedQuarterTrialPlan(folder/'request.json');r=validate_proof(p,folder/'screen.json')
print(json.dumps(dict(accepted=True,elapsed_seconds=r['elapsed_seconds'],rows=len(p.plan(p.source_position())),model=validate_experiment_basis()),ensure_ascii=False))
