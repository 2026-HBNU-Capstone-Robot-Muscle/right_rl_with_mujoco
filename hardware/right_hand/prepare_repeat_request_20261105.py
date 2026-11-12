"""Bind the new read-only OFF snapshot to the unchanged forward/hold route."""
import json,hashlib
from pathlib import Path
from forward_hold_plan_20261104 import ForwardHoldPlan
from hand_motor_mapping import SIM_TO_MOTOR_IDS
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/preintegration/repeat'
def main():
    src=R/'resume_off_readonly_20261105_033008_173180/result.json';r=json.loads(src.read_bytes())
    if r['status']!='readonly_data_saved' or not r['all_torque_off_read'] or r['maximum_drift_ticks']>2 or r['servo_writes'] or r['torque_changes'] or r.get('pose_health_validation_errors'):raise RuntimeError('OFF reference failed')
    O.mkdir(parents=True,exist_ok=True);p=O/'request.json'
    if p.exists():raise RuntimeError('Already frozen')
    p.write_text(json.dumps(dict(plan_version=ForwardHoldPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],source=src.relative_to(R).as_posix(),source_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
    plan=ForwardHoldPlan(p);rows=plan.plan(plan.source_position());print(json.dumps(dict(goals=len(rows),seconds=len(rows)*.075,read_only_source=str(src),devices_opened=False)))
if __name__=='__main__':main()
