"""Build an offline request from the already acquired stable OFF readings."""
import hashlib,json
from recovered_forward_plan_20261026 import RecoveredForwardPlan,FIXTURE,ROOT
from hand_motor_mapping import SIM_TO_MOTOR_IDS
def main():
    p=ROOT/'finish_empty_readonly_20261026_035633_554408/result.json';r=json.loads(p.read_bytes())
    if r['status']!='off_snapshots_saved' or r['maximum_drift_ticks']>2 or r['servo_write_commands_sent']:raise ValueError('Stable read-only source required')
    FIXTURE.mkdir(exist_ok=True)
    request=dict(plan_version=RecoveredForwardPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),origin='assistant_readonly_stable_off_reads',
        off_state=r['snapshots'][-1],readonly_source=p.relative_to(ROOT).as_posix(),readonly_source_sha256=hashlib.sha256(p.read_bytes()).hexdigest())
    (FIXTURE/'request.json').write_text(json.dumps(request,indent=2),encoding='utf-8')
    plan=RecoveredForwardPlan(FIXTURE/'request.json');print('Offline request ready; goal count',len(plan.plan(plan.source_position())))
if __name__=='__main__':main()
