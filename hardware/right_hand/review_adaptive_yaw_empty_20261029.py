"""Review the actual run without attributing an improvement to an inactive hook."""
import hashlib,json
from pathlib import Path
import numpy as np
from adaptive_yaw_empty_plan_20261029 import AdaptiveYawEmptyPlan
ROOT=Path(__file__).resolve().parent;FOLDER=ROOT/'adaptive_yaw_empty_20261029_062809_665980'
path=FOLDER/'result.json';r=json.loads(path.read_bytes())
p=AdaptiveYawEmptyPlan(FOLDER/'fresh_off_request.json');rows=p.plan(p.source_position())
written=[e['requested'] for e in r['write_events'] if e['address']==116]
assert written[-len(rows):]==[q.tolist() for phase,q in rows]
events=r.get('root_lag_wait_events',[]);yaw=[s for s in r['samples'] if s['phase']=='stage_7_horizontal_yaw']
out=dict(status=r['status'],source=path.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
    completed=r['status']=='adaptive_empty_yaw_observation_complete',cube_present=r['cube_present'],
    all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],
    all_actual_goal_writes_match_reviewed_queue=True,actual_queue_goals=len(rows),yaw_samples=len(yaw),
    root_wait_events=len(events),root_wait_seconds=r.get('root_lag_wait_total_seconds',0),
    actual_wait_controller_activation_verified=bool(events),improvement_cannot_be_attributed_to_inactive_wait=not bool(events),
    maximum_tracking_rad=max(max(s['tracking_rad']) for s in r['samples']),
    ID12_yaw_max_tracking_rad=max(s['tracking_rad'][11] for s in yaw),ID12_yaw_peak_current_ma=max(abs(s['current_ma'][11]) for s in yaw),
    current_peak_ma=max(max(abs(c) for c in s['current_ma']) for s in r['samples']),
    minimum_voltage_v=min(min(s['voltage_v']) for s in r['samples']),camera=r['camera_recording'],
    recovered_read_failures=len(r.get('read_failures',[])),recovery_durations_ms=[e['elapsed_ms'] for e in r.get('read_recovery_confirmation_events',[])],
    final_OFF_endpoints_not_a_stable_source=not r.get('final_off_reference_valid',False),cause_not_identified=True,assistant_motor_writes=0)
folder=FOLDER/'offline_review';folder.mkdir(exist_ok=True)
(folder/'review.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
print(json.dumps(out,ensure_ascii=False))
