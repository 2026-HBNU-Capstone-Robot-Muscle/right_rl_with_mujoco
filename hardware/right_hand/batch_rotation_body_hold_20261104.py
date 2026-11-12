"""Small offline continuation search. Results remain research, never admission."""
import json,time
import numpy as np
import mujoco
from research_rotation_body_hold_20261104 import CP,OUT,propose,evaluate,motor

def main():
    m=mujoco.MjModel.from_binary_path(str(CP/'closest_model.mjb'));z=np.load(CP/'closest_state.npz')
    up=z['up'];forward=np.array([up[2],0,-up[0]])
    cases=[]
    for lift in (2.,4.,6.):
        for shift in (-3.,0.,3.):
            cases.append((23.4,8.,10.,up*lift+forward*shift))
    cases.extend((0.,h,10.,np.zeros(3)) for h in (4.,12.,16.))
    results=[]
    for idx,(turn,lower,w,move) in enumerate(cases):
        name=f'translation_case_{idx:02d}';O=OUT/name;O.mkdir(parents=True,exist_ok=True)
        began=time.perf_counter();rows,ik,stop=propose(m,z,turn,lower,w,move)
        report=dict(name=name,turn_target_deg=turn,thumb_lower_mm=lower,planned_translation_mm=move.tolist(),geometry_stop=stop,IK=ik,hardware_execution_allowed=False,physical90_verified=False,devices_opened=False)
        if rows is not None:
            np.savez_compressed(O/'research_suffix.npz',start=motor(z['ctrl']),goals=rows)
            report['dynamic']=evaluate(m,z,rows,O/'end_state.npz')
            report['suffix_samples']=len(rows)
        report['elapsed_seconds']=time.perf_counter()-began
        (O/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        results.append(report)
        (OUT/'translation_batch.json').write_text(json.dumps(results,indent=2)+'\n',encoding='utf-8')
        print(json.dumps({k:v for k,v in report.items() if k!='IK'}),flush=True)
if __name__=='__main__':main()
