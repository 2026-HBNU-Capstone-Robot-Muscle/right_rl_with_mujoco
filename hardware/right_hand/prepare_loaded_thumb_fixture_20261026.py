"""Current stable OFF readings become an offline reference; no devices."""
import json
from loaded_thumb_rotation_plan_20261026 import ROOT,FIXTURE,LoadedThumbRotationPlan
def main():
    source=ROOT/'finish_empty_readonly_20261026_130555_407292/result.json';r=json.loads(source.read_bytes())
    if r['status']!='off_snapshots_saved' or r['maximum_drift_ticks']>2:raise ValueError('Unstable current OFF reference')
    request=dict(plan_version=LoadedThumbRotationPlan.PLAN_VERSION,motor_ids_in_sim_order=[8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1],
        off_state=r['snapshots'][-1],readonly_source=source.relative_to(ROOT).as_posix())
    FIXTURE.mkdir(exist_ok=True);path=FIXTURE/'request.json';path.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8')
    p=LoadedThumbRotationPlan(path);print(dict(goals=len(p.plan(p.source_position())),nominal_seconds=len(p.plan(p.source_position()))*p.SAMPLE_PERIOD,on_timeout=p.ON_TIMEOUT))
if __name__=='__main__':main()
