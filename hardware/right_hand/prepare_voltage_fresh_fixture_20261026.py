"""Use saved verified post-OFF readings as an offline reference, no devices."""
import json,hashlib
from thumb_clear_root_empty_plan_20261026 import ROOT,FIXTURE,ThumbRootEmptyPlan
def main():
    source=sorted(ROOT.glob('finish_empty_readonly_20261026_*/result.json'))[-1];r=json.loads(source.read_bytes())
    snapshots=r.get('snapshots',[])
    if r.get('status')!='off_snapshots_saved' or r.get('maximum_drift_ticks',999)>2 or len(snapshots)!=3:
        raise ValueError('Fresh read-only reference incomplete or moving')
    state=snapshots[-1]
    print('STATE',json.dumps(state,ensure_ascii=False))
    ThumbRootEmptyPlan.validate_off_state(state)
    request=dict(plan_version=ThumbRootEmptyPlan.PLAN_VERSION,
        motor_ids_in_sim_order=[8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1],off_state=state,
        origin='stable_readonly_REFERENCE_only_actual_start_rechecked_before_writes',
        readonly_source=source.relative_to(ROOT).as_posix(),readonly_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    FIXTURE.mkdir(exist_ok=True)
    path=FIXTURE/'request.json';path.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8')
    p=ThumbRootEmptyPlan(path);print('READY',json.dumps(dict(goals=len(p.plan(p.source_position())),source_position=p.source_position().tolist())))
if __name__=='__main__':main()
