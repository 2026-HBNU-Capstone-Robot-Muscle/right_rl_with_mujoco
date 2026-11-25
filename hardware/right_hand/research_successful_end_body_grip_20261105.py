"""Lower contacts after the same successful reference; offline proposals only."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import json,time,hashlib
from pathlib import Path
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import propose,evaluate,restore,inspect_for,motor
from audit_upper_layer_clearance_20261104 import local_mesh,box_gap
from audit_upper_layer_sweep_20261104 import sweep

R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/preintegration/body_grip';CP=O.parent/'reference_end'

def audit(m,z):
    d=restore(m,z);cube=m.geom('cube').id;hand,_,_=inspect_for(m);rows=[]
    for g in range(m.ngeom):
        if int(m.geom_bodyid[g]) not in hand or not(m.geom_contype[g] or m.geom_conaffinity[g]):continue
        v=local_mesh(m,d,g,cube)
        if v is None:rows.append(dict(geom=m.geom(g).name,handled=False));continue
        gap=box_gap(v,np.array([0.,-55/3,0]),np.array([27.5,55/6,27.5]))
        rows.append(dict(geom=m.geom(g).name,handled=True,static_upper_gap_mm=gap))
    bad=[r for r in rows if not r.get('handled') or r['static_upper_gap_mm']<=0]
    return dict(geoms=len(rows),static_upper_all_clear=not bad,interfering=bad)

def main():
    O.mkdir(parents=True,exist_ok=True);m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb'));z=np.load(CP/'end_state.npz');results=[]
    base=audit(m,z);(O/'reference_clearance.json').write_text(json.dumps(base,indent=2)+'\n',encoding='utf-8');print('REFERENCE',json.dumps(base),flush=True)
    for mid,thumb in ((8,8),(12,12),(16,12),(20,16),(24,16),(24,20)):
      for weight in (10.,3.):
        name=f'lower_m{mid}_t{thumb}_n{weight:g}';out=O/name;out.mkdir(exist_ok=True);began=time.perf_counter()
        rows,ik,error=propose(m,z,0.,thumb,weight,middle_lower_mm=mid)
        row=dict(name=name,middle_lower_mm=mid,thumb_lower_mm=thumb,normal_weight=weight,geometry_stop=error,IK=ik,hardware_execution_allowed=False,devices_opened=False)
        if rows is not None:
            np.savez_compressed(out/'suffix.npz',start=motor(z['ctrl']),goals=rows)
            row['dynamic']=evaluate(m,z,rows,out/'end_state.npz');row['goals']=len(rows)
            if row['dynamic']['safety_passed']:row['clearance']=audit(m,np.load(out/'end_state.npz'))
        row['elapsed_seconds']=time.perf_counter()-began;results.append(row);(out/'review.json').write_text(json.dumps(row,indent=2)+'\n',encoding='utf-8')
        report=dict(status='postturn_lower_body_grip_research',source_candidate='right_hand_rotation_20261104/forward_hold/candidate.npz',source_simulation_yaw_deg=json.loads((CP/'review.json').read_bytes())['cases'][0]['final_yaw_from_placement_deg'],physical_source_approximately_quarter_turn=True,model_is_not_registered_to_physical_cube=True,cube_not_reset_in_suffix=True,physics_guards_unchanged=True,hardware_execution_allowed=False,devices_opened=False,cases=results)
        (O/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        print(json.dumps({k:v for k,v in row.items() if k!='IK' and k!='dynamic'}|{'dynamic':None if 'dynamic' not in row else {k:v for k,v in row['dynamic'].items() if k!='trace'}}),flush=True)
if __name__=='__main__':main()
