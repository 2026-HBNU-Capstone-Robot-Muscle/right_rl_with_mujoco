import json,sys
from pathlib import Path
from fresh_empty_grasp_plan_20261027 import PROOF
r=json.loads((Path(sys.argv[1]) if len(sys.argv)>1 else PROOF).read_bytes());summary=dict(accepted=r['accepted_for_empty_only'],code_unchanged=r['code_unchanged'],plan_version=r.get('plan_version'),elapsed_seconds=r['elapsed_seconds'])
for key in ('static_screens','dynamic_sensitivity','camera_gap_hold_sensitivity'):
    rows=r[key];failed=[v for v in rows if not v['passed']];summary[key]=dict(total=len(rows),passed=sum(v['passed'] for v in rows),failed_fronts=sorted({v['book_front_mm'] for v in failed}),failed_examples=failed[:1])
print(json.dumps(summary,indent=2))
