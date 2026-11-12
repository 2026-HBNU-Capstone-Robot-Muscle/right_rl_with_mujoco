"""Assess remaining plan length from fresh OFF readings, no devices."""
import json,tempfile
from pathlib import Path
from thumb_clear_root_empty_plan_20261026 import ThumbRootEmptyPlan,ROOT
def main():
    source=ROOT/'finish_empty_readonly_20261026_130555_407292/result.json';r=json.loads(source.read_bytes())
    state=r['snapshots'][-1]
    request=dict(plan_version=ThumbRootEmptyPlan.PLAN_VERSION,motor_ids_in_sim_order=[8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1],off_state=state)
    with tempfile.TemporaryDirectory() as folder:
        path=Path(folder)/'request.json';path.write_text(json.dumps(request),encoding='utf-8')
        p=ThumbRootEmptyPlan(path);p.ON_TIMEOUT=300
        try:
            rows=p.plan(p.source_position())
            from collections import Counter
            print(json.dumps(dict(goals=len(rows),nominal_seconds=len(rows)*p.SAMPLE_PERIOD,phases=dict(Counter(phase for phase,q in rows)))))
        except Exception as exc:print(type(exc).__name__,str(exc))
if __name__=='__main__':main()
