"""Version replay of current actual EMPTY observation; no modified encoders."""
import hashlib
import json
from book_grasp_trial_plan import ROOT,BookGraspPlan
source=ROOT/'book_settled_off_readonly_20261024_180453_383542/request.json'
data=source.read_bytes();request=json.loads(data)
request.update(plan_version=BookGraspPlan.PLAN_VERSION,
    origin='offline_version_replay_of_actual_stable_EMPTY_OFF_snapshot',
    original_request=source.relative_to(ROOT).as_posix(),original_request_sha256=hashlib.sha256(data).hexdigest(),
    hardware_execution_allowed=False)
folder=ROOT/'right_hand_rotation_20261024/book20_current_review/grasp_route_fixture_v5';folder.mkdir(exist_ok=True)
path=folder/'request.json'
if path.exists() and json.loads(path.read_bytes())!=request:raise ValueError('Different fixture exists')
path.write_text(json.dumps(request,indent=2),encoding='utf-8')
plan=BookGraspPlan(path);rows=plan.plan(plan.source_position())
print(json.dumps(dict(request=str(path),command_count=len(rows),final_goal=rows[-1][1].tolist())))
