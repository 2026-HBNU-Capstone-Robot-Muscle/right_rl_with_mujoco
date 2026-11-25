"""Lower both pinching contacts without changing force or model parameters."""
import argparse,json,time
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import CP,OUT,propose,evaluate,motor

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--deep',action='store_true');args=ap.parse_args()
    m=mujoco.MjModel.from_binary_path(str(CP/'closest_model.mjb'));z=np.load(CP/'closest_state.npz');results=[]
    for middle in ((16.,20.,24.) if args.deep else (4.,8.,12.)):
      for thumb in ((16.,) if args.deep else (12.,16.)):
        O=OUT/f'lower_pair_m{middle:g}_t{thumb:g}';O.mkdir(parents=True,exist_ok=True);began=time.perf_counter()
        rows,ik,error=propose(m,z,0.,thumb,10.,middle_lower_mm=middle)
        report=dict(middle_lower_mm=middle,thumb_lower_mm=thumb,geometry_stop=error,IK=ik,hardware_execution_allowed=False,devices_opened=False)
        if rows is not None:
            report['dynamic']=evaluate(m,z,rows,O/'end_state.npz')
            np.savez_compressed(O/'research_suffix.npz',start=motor(z['ctrl']),goals=rows)
            report['goals']=len(rows)
        report['elapsed_seconds']=time.perf_counter()-began
        (O/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');results.append(report)
        (OUT/('lower_pair_deep_batch.json' if args.deep else 'lower_pair_batch.json')).write_text(json.dumps(results,indent=2)+'\n',encoding='utf-8')
        print(json.dumps({k:v for k,v in report.items() if k!='IK'}),flush=True)
if __name__=='__main__':main()
