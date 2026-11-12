"""Label an unchanged historical EMPTY snapshot for the shortened route replay."""
import hashlib
import json
from pathlib import Path
from book_grasp_trial_plan import ROOT,BookGraspPlan

source=ROOT/'book_cube_grasp_trial_20261024_172726_286561/fresh_off_request.json'
data=source.read_bytes();request=json.loads(data)
request.update(plan_version=BookGraspPlan.PLAN_VERSION,
    origin='offline_version_replay_of_user_saved_stable_EMPTY_OFF_snapshot',
    original_request=source.relative_to(ROOT).as_posix(),original_request_sha256=hashlib.sha256(data).hexdigest(),
    hardware_execution_allowed=False)
folder=ROOT/'right_hand_rotation_20261024/book20_current_review/grasp_route_fixture_v3';folder.mkdir(exist_ok=True)
path=folder/'request.json'
if path.exists() and json.loads(path.read_bytes())!=request:raise ValueError('Refuse to overwrite a different source')
path.write_text(json.dumps(request,indent=2),encoding='utf-8')
plan=BookGraspPlan(path);rows=plan.plan(plan.source_position())
print(json.dumps(dict(request=str(path),command_count=len(rows),final_goal=rows[-1][1].tolist())))
