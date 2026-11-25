import json
from horizontal_quarter_plan_20261027 import OUT
r=json.loads((OUT/'ring_side_diagnostic/review.json').read_bytes())
for result in r['results']:
    clears=[v['clear'] for v in result['cases'] if v['clear']]
    print(json.dumps(dict(delta=result['delta_ticks'],max_clear_sample=max((v['sample'] for v in clears),default=None),ring_clear_bounds=[min((v['ring_goal'] for v in clears),default=None),max((v['ring_goal'] for v in clears),default=None)],accepted_geometric_candidate=result['all_cleared'] and result['max_increase_mm']<=1e-6 and result['after_clearance_self_mm']<=1e-6 and result['other_self_mm']<=1e-6)))
