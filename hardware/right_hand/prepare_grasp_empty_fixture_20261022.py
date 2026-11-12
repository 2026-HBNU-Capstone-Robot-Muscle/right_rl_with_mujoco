"""Build a saved OFF-state fixture, explicitly not a current stable start."""
import json
from pathlib import Path
from grasp_approach_plan import RECOVERY_SOURCES,GraspApproachPlan
from thumb_live_start_gate import write_request
ROOT=Path(__file__).resolve().parent
out=ROOT/'prepared_cube_review_20261022/empty_approach_v2_fixture';out.mkdir(exist_ok=True)
r=json.loads((ROOT/RECOVERY_SOURCES[-1][0]).read_bytes())
s=r['preflight_state']
request=write_request(out,s,GraspApproachPlan.PLAN_VERSION,GraspApproachPlan.validate_off_state)
doc=json.loads(request.read_bytes());doc['origin']='saved_relaxing_off_fixture_not_live'
request.write_text(json.dumps(doc,indent=2),encoding='utf-8')
request=out/'request.json';request.write_text(json.dumps(doc,indent=2),encoding='utf-8')
p=GraspApproachPlan(request)
print('SAVED relaxing OFF sample, not live or verified stationary. Goals:',len(p.plan(p.source_position())))
print(request)
