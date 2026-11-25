"""Refine EMPTY support angle hypotheses; continuous free-cube models only."""
import json,time
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import R,CP,OUT,evaluate,native
from replay_body_hold_from_placement_20261104 import initial

def blend(a,b):
    n=max(1,int(np.ceil(max(abs(b-a))/2)))
    return np.rint([a+(b-a)*i/n for i in range(1,n+1)]).astype(int)

def main():
    old=np.load(R/'right_hand_rotation_20261030/single_forward/candidate.npz');z=np.load(CP/'closest_state.npz');seed=np.array(json.loads((R/'right_hand_rotation_20261030/stable_support_research/natural_recess_hypothesis_seed.json').read_bytes())['cube_qpos']);results=[]
    for height in (128,144,152,168,176):
        began=time.perf_counter();O=OUT/f'fine_support_{height}';O.mkdir(exist_ok=True)
        start=old['start'].copy();start[[7,11]]+=height;goal=old['start'].copy();goal[7]=start[7]
        first=blend(start,goal);source=old['goals'][:int(old['forward_end_sample'])].copy();source[:,7]=start[7];values=np.vstack([first,source,np.tile(source[-1],(67,1))])
        m=mujoco.MjModel.from_binary_path(str(CP/'closest_model.mjb'));lo,hi,nlo,nhi=native(m)
        assert np.all(values>=lo) and np.all(values<=hi) and max(abs(np.diff(np.vstack([start,values]),axis=0)).ravel())<=2
        state,placement=initial(m,start,z['up'],seed)
        report=dict(raise_ticks=height,initial_placement=placement,hardware_execution_allowed=False,devices_opened=False)
        if placement['passed']:
            report['dynamic']=evaluate(m,state,values,O/'end_state.npz');np.savez_compressed(O/'candidate.npz',start=start,goals=values)
        report['elapsed_seconds']=time.perf_counter()-began;results.append(report)
        (O/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');(OUT/'fine_support_sweep.json').write_text(json.dumps(results,indent=2)+'\n',encoding='utf-8')
        print(json.dumps({k:v for k,v in report.items() if k!='dynamic'}|{'dynamic':None if 'dynamic' not in report else {k:v for k,v in report['dynamic'].items() if k!='trace'}}),flush=True)
if __name__=='__main__':main()
