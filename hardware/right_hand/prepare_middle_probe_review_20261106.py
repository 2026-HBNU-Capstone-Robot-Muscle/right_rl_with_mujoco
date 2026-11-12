"""Freeze the latest OFF reference before the new probe's current-pose screen."""
import json
from observed_book_model_policy import healthy_off
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from middle_probe_plan_20261106 import R,O,REQUEST,PROOF,MiddleProbePlan,sha,validate_candidate
def main():
    source=R/'resume_off_readonly_20261106_220704_740850/result.json';r=json.loads(source.read_bytes())
    if r['status']!='readonly_data_saved' or r['servo_writes'] or r['torque_changes'] or not r.get('all_torque_off_read') or len(r['snapshots'])!=3 or r['maximum_drift_ticks']>2:raise RuntimeError('Incomplete fresh OFF source')
    for s in r['snapshots']:healthy_off(s)
    validate_candidate()
    request=dict(plan_version=MiddleProbePlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],source=source.relative_to(R).as_posix(),source_sha256=sha(source),hardware_execution_allowed=False,user_confirmed_cube_removed=True)
    if PROOF.exists():raise RuntimeError('Refuse to overwrite frozen probe review')
    if REQUEST.exists():
        if json.loads(REQUEST.read_bytes())!=request:raise RuntimeError('Frozen probe request differs')
    else:REQUEST.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8')
    plan=MiddleProbePlan(REQUEST);rows=plan.plan(plan.source_position());print(json.dumps(dict(request=REQUEST.relative_to(R).as_posix(),whole_goals=len(rows),nominal_motion_seconds=len(rows)*.075,devices_opened=False,hardware_execution_allowed=False)),flush=True)
if __name__=='__main__':main()
