"""Current confirmed-empty OFF source for an offline slow-path replay."""
import hashlib,json
from slow_forward_plan_20261026 import SlowForwardPlan,ROOT,FIXTURE
from hand_motor_mapping import SIM_TO_MOTOR_IDS
def main():
    source=ROOT/'finish_empty_readonly_20261026_043436_613651/result.json';r=json.loads(source.read_bytes())
    if r['status']!='off_snapshots_saved' or r['maximum_drift_ticks']>2 or r['servo_write_commands_sent']:
        raise ValueError('Stable read-only source required')
    FIXTURE.mkdir(exist_ok=True)
    request=dict(plan_version=SlowForwardPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
        origin='assistant_readonly_after_operator_confirmed_cube_removed',off_state=r['snapshots'][-1],
        readonly_source=source.relative_to(ROOT).as_posix(),readonly_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    (FIXTURE/'request.json').write_text(json.dumps(request,indent=2),encoding='utf-8')
    p=SlowForwardPlan(FIXTURE/'request.json');rows=p.plan(p.source_position())
    print(json.dumps(dict(goals=len(rows),nominal_motion_seconds=len(rows)*p.SAMPLE_PERIOD,
        loaded_reference_seconds=760*p.SAMPLE_PERIOD,loaded_finish_seconds=64*p.SAMPLE_PERIOD,
        sample_period=p.SAMPLE_PERIOD,on_timeout=p.ON_TIMEOUT,motor_writes=0)))
if __name__=='__main__':main()
