"""Prepare the fresh OFF reference after a clean camera/serial observation."""
import hashlib,json
import numpy as np
from horizontal_quarter_plan_20261027 import ROOT,OUT,HorizontalQuarterPlan
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from observed_book_model_policy import healthy_off
source=ROOT/'usb_swap_readonly_20261028_044716_809381/result.json';r=json.loads(source.read_bytes())
assert r['status']=='swapped_usb_readonly_observation_complete' and r['baud']==1000000
assert r['motor_writes']==0 and r['torque_changes']==0 and not r['errors']
assert r['reads']>=700 and r['camera_frames']==r['usable_frames']>=500 and r['blank_frames']==0
assert r['camera_observed_duration_s']>=55 and r['maximum_frame_gap_s']<.5
assert len(r['individual_checks'])==16
states=r['last_snapshots'];assert len(states)==3
for state in states:healthy_off(state)
drift=int(np.ptp([s['position'] for s in states],axis=0).max());assert drift<=2
dest=OUT/'after_usb_swap_044716';dest.mkdir(exist_ok=True)
request=dict(plan_version=HorizontalQuarterPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
    origin='after_user_swapped_usb_ports_clean60s_camera_and_paced_bus_observation',
    source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    off_state=states[-1],hardware_execution_allowed=False)
path=dest/'request.json';path.write_bytes((json.dumps(request,indent=2)+'\n').encode())
plan=HorizontalQuarterPlan(path)
print(json.dumps(dict(status='new_usb_reference_ready_model_pending',position=plan.source_position().tolist(),
    drift_ticks=drift,goals=len(plan.plan(plan.source_position())),
    camera_images=[s['file'] for s in r['camera_samples']],motor_writes=0)))
