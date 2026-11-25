import json,hashlib,shutil
import numpy as np
from quarter_finish_empty_plan_20261025 import QuarterFinishEmptyPlan,ROOT,FIXTURE,empty_basis,FINISH_PHASE,TURN_PHASE
from hand_motor_mapping import SIM_TO_MOTOR_IDS
def main():
    source=ROOT/'finish_empty_readonly_20261025_230039_588520/result.json';r=json.loads(source.read_bytes())
    if r['status']!='off_snapshots_saved' or r['maximum_drift_ticks']>2 or r['servo_write_commands_sent']!=0:raise ValueError('No stable OFF source')
    FIXTURE.mkdir(exist_ok=True);p=FIXTURE/'request.json'
    request=dict(plan_version=QuarterFinishEmptyPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),origin='assistant_readonly_stable_off_reads',off_state=r['snapshots'][-1],readonly_source=source.relative_to(ROOT).as_posix(),readonly_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    p.write_text(json.dumps(request,indent=2),encoding='utf-8');plan=QuarterFinishEmptyPlan(p);rows=plan.plan(plan.source_position());basis=empty_basis()
    print(json.dumps(dict(rows=len(rows),nominal_motion_seconds=len(rows)*.05,reference_count=sum(t==TURN_PHASE for t,q in rows),finish_count=sum(t==FINISH_PHASE for t,q in rows),last_goal=rows[-1][1].tolist(),basis=basis)),flush=True)
if __name__=='__main__':main()
