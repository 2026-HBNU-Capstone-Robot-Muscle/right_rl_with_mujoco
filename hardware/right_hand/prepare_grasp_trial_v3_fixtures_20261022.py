"""Archive three actual saved OFF inputs for opening-path regression screens."""
import json
import numpy as np
from grasp_place_plan import ROOT,GraspPlacePlan,healthy_off
from thumb_live_start_gate import write_request

latest=json.loads((ROOT/'cube_grasp_trial_20261022_024003_945968/result.json').read_bytes())
first=json.loads((ROOT/'cube_grasp_trial_20261022_023904_356527/result.json').read_bytes())
cases=[('grasp_trial_v3_fixture',latest['preflight_state']),
       ('grasp_trial_v3_after_stop_fixture',latest['final_off_snapshots'][-1]),
       ('grasp_trial_v3_first_stop_fixture',first['preflight_state'])]
for name,state in cases:
    folder=ROOT/'prepared_cube_review_20261022'/name;folder.mkdir(exist_ok=True)
    state={k:np.asarray(v) for k,v in state.items()}
    path=write_request(folder,state,GraspPlacePlan.PLAN_VERSION,healthy_off)
    request=folder/'request.json';request.write_bytes(path.read_bytes())
    plan=GraspPlacePlan(request);rows=plan.plan(plan.source_position())
    print(name,dict(goals=len(rows),motion_s=len(rows)*.05,hardware_commands_sent=0))
