"""Version-labelled replay of unchanged stable OFF data, never motor commands."""
import hashlib
import json
from book_small_yaw_plan import ROOT,BookSmallYawPlan

source=ROOT/'book_settled_off_readonly_20261024_213523_920943/request.json'
data=source.read_bytes();request=json.loads(data)
request.update(plan_version=BookSmallYawPlan.PLAN_VERSION,
    origin='offline_version_replay_of_actual_stable_EMPTY_OFF_snapshot',
    original_request=source.relative_to(ROOT).as_posix(),original_request_sha256=hashlib.sha256(data).hexdigest(),
    hardware_execution_allowed=False)
folder=ROOT/'right_hand_rotation_20261024/book20_current_review/small_yaw_fixture_v1'
folder.mkdir(exist_ok=True);path=folder/'request.json'
if path.exists() and json.loads(path.read_bytes())!=request:raise ValueError('Different fixture exists')
path.write_text(json.dumps(request,indent=2),encoding='utf-8')
plan=BookSmallYawPlan(path);rows=plan.plan(plan.source_position())
print(json.dumps(dict(request=str(path),command_count=len(rows),
    delayed_cycle_budget_seconds=(len(rows)+40)*.06+2,final_goal=rows[-1][1].tolist())))
