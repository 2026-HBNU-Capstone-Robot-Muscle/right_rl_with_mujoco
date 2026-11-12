"""Validate the saved completed EMPTY run against its exact issued goal queue."""
import hashlib,json
from collections import Counter
from pathlib import Path
import numpy as np
from thumb_clear_root_empty_plan_20261026 import ThumbRootEmptyPlan
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'thumb_clear_root_empty_20261026_125659_859144'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    r=json.loads((SOURCE/'result.json').read_bytes());samples=r['samples']
    request=SOURCE/'fresh_off_request.json';p=ThumbRootEmptyPlan(request)
    initial=np.asarray(r['accepted_enable_goal']);rows=p.plan(initial)
    assert r['status']==p.STATUS and r['all_torque_off_verified'] and r['settings_restored']
    assert len(samples)==len(rows)+20
    previous=np.vstack([initial]+[q for _,q in rows[:-1]])
    np.testing.assert_array_equal([s['goal'] for s in samples[:len(rows)]],previous)
    np.testing.assert_array_equal([s['goal_readback'] for s in samples],[s['goal'] for s in samples])
    np.testing.assert_array_equal([s['goal'] for s in samples[len(rows):]],np.tile(rows[-1][1],(20,1)))
    assert [s['phase'] for s in samples[:len(rows)]]==[phase for phase,q in rows]
    np.testing.assert_array_equal(r['final_command'],rows[-1][1])
    frames=json.loads((SOURCE/'camera/frames.json').read_bytes())['frames']
    out=dict(accepted=True,scope='EMPTY actual motion only',source=SOURCE.name,source_sha256=sha(SOURCE/'result.json'),
        request_sha256=sha(request),plan_version=p.PLAN_VERSION,goal_digest=p.plan_digest(initial),
        planned_goals=len(rows),recorded_samples=len(samples),phases=dict(Counter(s['phase'] for s in samples)),
        exact_saved_goal_readbacks=True,exact_ordered_goal_queue=True,all_torque_off_verified=True,settings_restored=True,
        minimum_voltage_v=min(min(s['voltage_v']) for s in samples),
        maximum_current_ma=max(max(abs(v) for v in s['current_ma']) for s in samples),
        maximum_tracking_rad=max(max(s['tracking_rad']) for s in samples),
        frame_count=len(frames),final_off_reference_valid=r['final_off_reference_valid'],
        physical_cube_rotation_verified=False,assistant_motor_writes=0)
    (SOURCE/'completion_review.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(out),flush=True)
if __name__=='__main__':main()
