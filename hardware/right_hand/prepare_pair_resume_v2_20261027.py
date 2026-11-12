"""Create the review request from the stopped run's actual READ-only start."""
import json
from new_pair_grasp_plan_20261027 import ROOT,OUT,REQUEST,NewPairGraspPlan
OUT.mkdir(exist_ok=True);source=ROOT/'new_pair_grasp_20261027_045408_418203/fresh_off_request.json';r=json.loads(source.read_bytes());r.update(plan_version=NewPairGraspPlan.PLAN_VERSION,review_origin=source.relative_to(ROOT).as_posix());REQUEST.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');plan=NewPairGraspPlan(REQUEST);rows=plan.plan(plan.source_position());print('Direct recovery/current source:',len(rows),'goals;',len(rows)*plan.SAMPLE_PERIOD,'s nominal; no devices opened.')
