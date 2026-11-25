"""Offline request construction from the authorized read-only evidence."""
import json
from fresh_empty_grasp_plan_20261027 import ROOT,CANONICAL,REQUEST,FreshEmptyGraspPlan
from hand_motor_mapping import SIM_TO_MOTOR_IDS
r=json.loads((CANONICAL/'result.json').read_bytes());motors=r['snapshots'][-1]['motors'];state={key:[m[field] for m in motors] for key,field in [('position','position_ticks_signed'),('voltage','voltage_raw'),('temperature','temperature_c'),('torque','torque_enabled'),('error','hardware_error'),('watchdog','bus_watchdog_raw')]};request=dict(plan_version=FreshEmptyGraspPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=state,source_readonly_evidence=(CANONICAL/'result.json').relative_to(ROOT).as_posix());REQUEST.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8');p=FreshEmptyGraspPlan(REQUEST);print('OFFLINE REQUEST',REQUEST);print('Goals',len(p.plan(p.source_position())),'No devices opened.')
