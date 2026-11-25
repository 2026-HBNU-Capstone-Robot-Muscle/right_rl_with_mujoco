"""Unmodified latest confirmed EMPTY OFF data, relabelled only for offline replay."""
import hashlib
import json
from book_dynamic_yaw_plan import ROOT,BookDynamicYawPlan
source=ROOT/'book_settled_off_readonly_20261024_220357_083205/request.json'
blob=source.read_bytes();r=json.loads(blob)
r.update(plan_version=BookDynamicYawPlan.PLAN_VERSION,origin='offline_version_replay_of_confirmed_EMPTY_OFF_source',
    original_request=source.relative_to(ROOT).as_posix(),original_request_sha256=hashlib.sha256(blob).hexdigest(),
    cube_removed_by_user_after_previous_trial=True,hardware_execution_allowed=False)
folder=ROOT/'right_hand_rotation_20261024/book20_current_review/dynamic_yaw_fixture_v2';folder.mkdir(exist_ok=True)
path=folder/'request.json'
if path.exists() and json.loads(path.read_bytes())!=r:raise ValueError('Different fixture exists')
path.write_text(json.dumps(r,indent=2),encoding='utf-8')
p=BookDynamicYawPlan(path);rows=p.plan(p.source_position())
print(json.dumps(dict(folder=str(folder),goals=len(rows),delayed_cycle_seconds=(len(rows)+40)*.06+2)))
