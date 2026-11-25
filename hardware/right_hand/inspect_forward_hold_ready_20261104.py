import json
from pathlib import Path
import numpy as np
from forward_hold_plan_20261104 import ROOT,OUT,REQUEST

def main():
    ref=np.array(json.loads(REQUEST.read_bytes())['off_state']['position']);latest=max(ROOT.glob('resume_off_readonly_20261104_*/result.json'),key=lambda p:p.stat().st_mtime);r=json.loads(latest.read_bytes());rows=[]
    if r.get('snapshots'):
        actual=np.array(r['snapshots'][-1]['position']);delta=actual-ref
        print(json.dumps(dict(latest_readonly=latest.parent.name,all_torque_off=r.get('all_torque_off_read'),drift_ticks=r.get('maximum_drift_ticks'),maximum_difference_from_request_ticks=int(max(abs(delta))),differences_over4=[(i+1,int(d)) for i,d in enumerate(delta) if abs(d)>4],pose_health_errors=r.get('pose_health_validation_errors'))))
    for front in (-15,-10,-5):
        p=OUT/f'material_front_{front}/review.json'
        if not p.exists():rows.append(dict(front=front,cases=0));continue
        d=json.loads(p.read_bytes());rows.append(dict(front=front,cases=len(d['cases']),all_passed=d['all_passed'],failed=[(c['gain_scale'],c['friction'],c['stop']) for c in d['cases'] if not c['passed']]))
    print(json.dumps(rows))
if __name__=='__main__':main()
