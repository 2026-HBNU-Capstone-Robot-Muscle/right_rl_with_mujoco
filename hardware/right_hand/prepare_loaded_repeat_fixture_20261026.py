"""Bind the newly captured stable OFF pose to the unchanged physical route."""
import hashlib,json
from loaded_repeat_plan_20261026 import ROOT,FIXTURE,LoadedRepeatPlan
from loaded_thumb_rotation_plan_20261026 import LoadedThumbRotationPlan
import numpy as np
def main():
    source=ROOT/'repeat_off_readonly_20261026_164534_888040/result.json';r=json.loads(source.read_bytes())
    assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']<=2 and r['servo_writes']==0
    request=dict(plan_version=LoadedRepeatPlan.PLAN_VERSION,motor_ids_in_sim_order=[8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1],
        off_state=r['snapshots'][-1],readonly_source=source.relative_to(ROOT).as_posix(),readonly_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    FIXTURE.mkdir(exist_ok=True);path=FIXTURE/'request.json';path.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8')
    p=LoadedRepeatPlan(path);raw=p.source_position();new=p.plan(raw);old=LoadedThumbRotationPlan.plan(p,raw)
    assert [phase for phase,q in new]==[phase for phase,q in old]
    np.testing.assert_array_equal([q for phase,q in new],[q for phase,q in old])
    print(dict(goals=len(new),nominal_seconds=len(new)*p.SAMPLE_PERIOD,goal_positions_order_and_pacing_unchanged=True))
if __name__=='__main__':main()
