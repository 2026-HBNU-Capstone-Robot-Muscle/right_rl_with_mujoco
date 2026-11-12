"""An offline reference fixture; fresh OFF state is screened again at execution."""
import hashlib,json
from thumb_clear_root_empty_plan_20261026 import ThumbRootEmptyPlan,ROOT,FIXTURE,proposal_metadata
from hand_motor_mapping import SIM_TO_MOTOR_IDS
def main():
    successful=[]
    for p in sorted(ROOT.glob('finish_empty_readonly_*/result.json')):
        r=json.loads(p.read_bytes())
        if r.get('status')=='off_snapshots_saved' and r.get('maximum_drift_ticks',99)<=2 and not r.get('servo_write_commands_sent'):
            successful.append((p,r))
    if not successful:raise ValueError('No successful read-only fixture')
    source,r=successful[-1];FIXTURE.mkdir(exist_ok=True)
    request=dict(plan_version=ThumbRootEmptyPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],
                 origin='stable_readonly_REFERENCE_only_actual_start_rechecked_before_writes',readonly_source=source.relative_to(ROOT).as_posix(),
                 readonly_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    (FIXTURE/'request.json').write_text(json.dumps(request,indent=2),encoding='utf-8')
    p=ThumbRootEmptyPlan(FIXTURE/'request.json');rows=p.plan(p.source_position())
    print(json.dumps(dict(source=str(source.relative_to(ROOT)),goals=len(rows),nominal_motion_seconds=len(rows)*p.SAMPLE_PERIOD,proposal=proposal_metadata(),current_hardware_contact_verified=False,hardware_writes=0)))
if __name__=='__main__':main()
