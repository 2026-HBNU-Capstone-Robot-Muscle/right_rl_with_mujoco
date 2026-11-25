import hashlib,json
from loaded_quarter_finish_plan_20261025 import LoadedQuarterFinishPlan,ROOT,FIXTURE,loaded_basis,TURN_PHASE,FINISH_PHASE
from hand_motor_mapping import SIM_TO_MOTOR_IDS
def main():
    source=ROOT/'finish_empty_readonly_20261025_235113_620117/result.json';r=json.loads(source.read_bytes())
    if r['status']!='off_snapshots_saved' or r['maximum_drift_ticks']>2 or r['servo_write_commands_sent']:raise ValueError('Stable OFF source required')
    FIXTURE.mkdir(exist_ok=True);p=FIXTURE/'request.json';request=dict(plan_version=LoadedQuarterFinishPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),origin='assistant_readonly_stable_off_reads',off_state=r['snapshots'][-1],readonly_source=source.relative_to(ROOT).as_posix(),readonly_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    p.write_text(json.dumps(request,indent=2),encoding='utf-8');plan=LoadedQuarterFinishPlan(p);rows=plan.plan(plan.source_position());basis=loaded_basis()
    print(json.dumps(dict(rows=len(rows),nominal_goal_seconds=len(rows)*.05,reference_goals=sum(x==TURN_PHASE for x,q in rows),finish_goals=sum(x==FINISH_PHASE for x,q in rows),loaded_basis=basis)),flush=True)
if __name__=='__main__':main()
