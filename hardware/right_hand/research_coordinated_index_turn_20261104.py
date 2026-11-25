"""Allow index support to move during a geometry-proposed remaining turn."""
import json,time
import numpy as np,mujoco
import research_rotation_body_hold_20261104 as s

def main():
    s.ACTIVE=np.array([0,1,2,3,4,5,6,7,12,13,14,15])
    m=mujoco.MjModel.from_binary_path(str(s.CP/'closest_model.mjb'));z=np.load(s.CP/'closest_state.npz');results=[]
    for normal in (2.,10.):
      for lift in (0.,2.):
       for middle in (0.,6.):
        name=f'coordinated_index_n{normal:g}_up{lift:g}_mf{middle:g}';O=s.OUT/name;O.mkdir(parents=True,exist_ok=True);began=time.perf_counter()
        rows,ik,error=s.propose(m,z,23.4,12.,normal,translation_mm=z['up']*lift,middle_lower_mm=middle)
        report=dict(name=name,normal_weight=normal,lift_mm=lift,middle_lower_mm=middle,geometry_stop=error,IK=ik,hardware_execution_allowed=False,devices_opened=False)
        if rows is not None:
            report['dynamic']=s.evaluate(m,z,rows,O/'end_state.npz')
            np.savez_compressed(O/'research_suffix.npz',start=s.motor(z['ctrl']),goals=rows)
            report['goals']=len(rows)
        report['elapsed_seconds']=time.perf_counter()-began
        (O/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');results.append(report)
        (s.OUT/'coordinated_index_turn_batch.json').write_text(json.dumps(results,indent=2)+'\n',encoding='utf-8')
        print(json.dumps({k:v for k,v in report.items() if k!='IK'}),flush=True)
if __name__=='__main__':main()
