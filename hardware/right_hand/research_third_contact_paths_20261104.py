"""Free-cube replay of several index-finger handoff orders; no hardware."""
import itertools,json,time
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import CP,OUT,motor,evaluate

def blend(a,b):
    n=max(1,int(np.ceil(max(abs(b-a))/2)))
    return np.rint([a+(b-a)*i/n for i in range(1,n+1)]).astype(int)

def main():
    m=mujoco.MjModel.from_binary_path(str(CP/'closest_model.mjb'));z=np.load(OUT/'turn_0_lower_8_normal_10/end_state.npz')
    geometry=json.loads((OUT/'postturn_third_contact_geometry.json').read_bytes())
    target=np.array(next(r['goal'] for r in geometry['reachable'] if r['finger']=='if'))
    start=motor(z['ctrl']);groups={'root':[7],'rot':[5],'bend':[4,6]}
    results=[]
    orders=[('together',)]+list(itertools.permutations(groups))
    for order in orders:
        O=OUT/('third_path_'+'_'.join(order));O.mkdir(parents=True,exist_ok=True);rows=[];previous=start.copy();began=time.perf_counter()
        if order==('together',):rows.extend(blend(previous,target))
        else:
            for phase in order:
                goal=previous.copy();goal[groups[phase]]=target[groups[phase]];rows.extend(blend(previous,goal));previous=goal
        rows.extend(target.copy() for _ in range(67));rows=np.array(rows)
        report=evaluate(m,z,rows,O/'end_state.npz');report.update(order=order,goals=len(rows),hardware_execution_allowed=False,devices_opened=False,elapsed_seconds=time.perf_counter()-began)
        np.savez_compressed(O/'research_suffix.npz',start=start,goals=rows)
        (O/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');results.append(report)
        (OUT/'third_contact_path_batch.json').write_text(json.dumps(results,indent=2)+'\n',encoding='utf-8')
        print(json.dumps({k:v for k,v in report.items() if k!='trace'}),flush=True)
if __name__=='__main__':main()
