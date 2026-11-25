"""Freeze the actual fresh OFF source and build its empty-only recovery proof."""
import argparse,json
from pathlib import Path
from observed_book_model_policy import healthy_off
from manual_grasp_plan_20261106 import R,OUT,REQUEST,PROOF,ManualGraspPlan,sha
from hand_motor_mapping import SIM_TO_MOTOR_IDS
def main():
    p=argparse.ArgumentParser();p.add_argument('source');a=p.parse_args();source=(R/a.source).resolve();source.relative_to(R);r=json.loads(source.read_bytes())
    if r.get('status')!='readonly_data_saved' or r.get('servo_writes') or r.get('torque_changes') or len(r.get('snapshots',[]))!=3 or not r.get('all_torque_off_read') or r.get('maximum_drift_ticks',999)>2:raise RuntimeError('Fresh OFF source incomplete')
    for state in r['snapshots']:healthy_off(state)
    if REQUEST.exists() or PROOF.exists():raise RuntimeError('Refuse to overwrite frozen teaching request/proof')
    request=dict(plan_version=ManualGraspPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],source=source.relative_to(R).as_posix(),source_sha256=sha(source),hardware_execution_allowed=False);REQUEST.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8')
    from screen_manual_grasp_current_20261106 import main as screen
    import sys
    saved=sys.argv
    try:sys.argv=['screen',str(REQUEST),str(PROOF)];screen()
    finally:sys.argv=saved
    plan=ManualGraspPlan(REQUEST);rows=plan.plan(plan.source_position());print(json.dumps(dict(goals=len(rows),nominal_motion_seconds=len(rows)*.075,automatic_open=False,joint_teaching_after_verified_OFF_only=True,devices_opened=False)))
if __name__=='__main__':main()
