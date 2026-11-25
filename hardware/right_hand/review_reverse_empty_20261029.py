"""Actual EMPTY reverse execution evidence; source goal reconstruction uses ON hold."""
import hashlib,json
from pathlib import Path
import numpy as np
from reverse_yaw_empty_plan_20261029 import ReverseYawEmptyPlan,ROOT
F=ROOT/'reverse_yaw_empty_20261029_164720_101745';r=json.loads((F/'result.json').read_bytes());p=ReverseYawEmptyPlan(F/'fresh_off_request.json')
rows=p.plan(np.asarray(r['accepted_enable_goal']));written=[e['requested'] for e in r['write_events'] if e['address']==116]
assert written[-len(rows):]==[q.tolist() for phase,q in rows]
assert r['status']=='empty_reverse_yaw_path_complete' and r['all_torque_off_verified'] and r['settings_restored'] and not r['cube_present']
samples=r['samples'];report=dict(status='actual_EMPTY_reverse_cycle_complete',source=(F/'result.json').relative_to(ROOT).as_posix(),
    source_sha256=hashlib.sha256((F/'result.json').read_bytes()).hexdigest(),all_actual_goal_writes_match_ON_rebased_queue=True,
    queue_goals=len(rows),cube_present=False,all_torque_off_verified=True,settings_restored=True,
    yaw_samples=sum(s['phase']=='stage_7_horizontal_yaw' for s in samples),
    minimum_voltage_v=min(min(s['voltage_v']) for s in samples),peak_current_ma=max(max(abs(c) for c in s['current_ma']) for s in samples),
    maximum_tracking_rad=max(max(s['tracking_rad']) for s in samples),read_failures=len(r.get('read_failures',[])),
    read_recoveries_confirmed=len(r.get('read_recovery_confirmation_events',[])),
    recovery_durations_ms=[x['elapsed_ms'] for x in r.get('read_recovery_confirmation_events',[])],
    camera_frames=r['camera_recording']['frame_count'],camera_blank_frames=r['camera_recording']['image_signal_blank_frames'],
    endpoint_OFF_reference_not_stable=not r.get('final_off_reference_valid',False),physical_inverse_cube_rotation_verified=False,assistant_motor_writes=0)
O=F/'offline_review';O.mkdir(exist_ok=True);(O/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report))
