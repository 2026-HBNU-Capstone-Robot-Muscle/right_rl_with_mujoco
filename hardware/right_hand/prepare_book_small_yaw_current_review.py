"""Replay the last actual stable OFF acquisition without modifying raw readings."""
import hashlib
import json
from book_small_yaw_plan import ROOT,BookSmallYawPlan

source=ROOT/'book_settled_off_readonly_20261024_215126_125157/request.json'
data=source.read_bytes();r=json.loads(data)
r.update(plan_version=BookSmallYawPlan.PLAN_VERSION,
    origin='offline_version_replay_of_actual_stable_EMPTY_OFF_snapshot',
    original_request=source.relative_to(ROOT).as_posix(),original_request_sha256=hashlib.sha256(data).hexdigest(),
    hardware_execution_allowed=False)
folder=ROOT/'right_hand_rotation_20261024/book20_current_review/small_yaw_current_215126'
folder.mkdir(exist_ok=True);path=folder/'request.json'
if path.exists() and json.loads(path.read_bytes())!=r:raise ValueError('Different review exists')
path.write_text(json.dumps(r,indent=2),encoding='utf-8')
p=BookSmallYawPlan(path);rows=p.plan(p.source_position())
print(json.dumps(dict(folder=str(folder),goals=len(rows),delayed_cycle_seconds=(len(rows)+40)*.06+2)))
