"""Create an OFF fixture from saved relaxation readings, not current hardware."""
import hashlib
import json
import numpy as np
from grasp_place_plan import ROOT,GraspPlacePlan,completed_source,healthy_off,endpoints
from thumb_live_start_gate import write_request

folder=ROOT/'prepared_cube_review_20261022/grasp_trial_v2_fixture';folder.mkdir(exist_ok=True)
source=ROOT/'cube_grasp_trial_20261022_023055_407697/result.json'
data=source.read_bytes();r=json.loads(data)
assert r['status']=='stopped' and r['failed_phase']=='preflight'
assert not r['write_events'] and r['all_torque_off_verified']
state={k:np.asarray(v) for k,v in r['preflight_state'].items()}
request=write_request(folder,state,GraspPlacePlan.PLAN_VERSION,healthy_off)
# Conventional offline fixture name for the integration test, never a current
# stable-start claim. Actual execution acquires three NEW stable OFF snapshots.
fixture=folder/'request.json';fixture.write_bytes(request.read_bytes())
plan=GraspPlacePlan(fixture);rows=plan.plan(plan.source_position())
print(json.dumps(dict(samples=len(rows),motion_seconds=len(rows)*.05,opened=endpoints()[0].tolist(),
    closed=endpoints()[1].tolist(),source_off_stable=False,
    source=str(source),source_sha256=hashlib.sha256(data).hexdigest(),
    source_fixture_only=True,hardware_commands_sent=0),indent=2))
