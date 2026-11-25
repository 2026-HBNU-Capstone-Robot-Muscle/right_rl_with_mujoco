import json
from pathlib import Path
from horizontal_quarter_plan_20261027 import HorizontalQuarterPlan,REQUEST,prefix_digest
ROOT=Path(__file__).resolve().parent
r=json.loads((ROOT/'right_hand_rotation_20261027/horizontal_quarter_trial_v3/empty_recovery_screen/review.json').read_bytes())
print(json.dumps(dict(first_static=r['static_cases'][0],worst_self=max(v['max_self_mm'] for v in r['static_cases']),worst_support=max(v['max_support_after_release_mm'] for v in r['static_cases']),max_initial_increase=max(v['initial_support_increase_mm'] for v in r['static_cases']),max_step_increase=max(v['step_support_increase_mm'] for v in r['static_cases']),root_clear_samples=sorted(set(v['root_clear_sample'] for v in r['static_cases'])),prefix_matches=r['prefix_sha256']==prefix_digest(HorizontalQuarterPlan(REQUEST))),indent=2))
