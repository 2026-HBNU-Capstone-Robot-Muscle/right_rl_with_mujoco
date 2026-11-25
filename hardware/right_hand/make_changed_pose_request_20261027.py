"""Build a separate request from the saved, torque-OFF current-pose attempt."""
import json
from fresh_empty_grasp_plan_20261027 import ROOT,FreshEmptyGraspPlan
source=ROOT/'fresh_empty_grasp_20261027_030819_048059/fresh_off_request.json';out=ROOT/'right_hand_rotation_20261026/changed_off_pose_20261027/request_v7.json';r=json.loads(source.read_bytes());r['plan_version']=FreshEmptyGraspPlan.PLAN_VERSION;r['source_saved_attempt']=source.relative_to(ROOT).as_posix();out.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');p=FreshEmptyGraspPlan(out);print(out);print('Goals',len(p.plan(p.source_position())),'No devices opened.')
