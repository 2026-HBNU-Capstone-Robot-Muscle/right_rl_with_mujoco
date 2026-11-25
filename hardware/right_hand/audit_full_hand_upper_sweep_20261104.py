"""Whole existing collision-hand clearance from a rotating upper-layer box."""
import json
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import CP,OUT,restore,inspect_for
from audit_upper_layer_clearance_20261104 import local_mesh
from audit_upper_layer_sweep_20261104 import sweep

def main():
    m=mujoco.MjModel.from_binary_path(str(CP/'closest_model.mjb'));path=OUT/'lower_pair_m20_t16/end_state.npz';d=restore(m,np.load(path));cube=m.geom('cube').id;hand,support,check=inspect_for(m);rows=[];unhandled=[]
    bound=float(2*27.5*np.sqrt(2)*np.sin(np.radians(1.)/4))
    for g in range(m.ngeom):
        if int(m.geom_bodyid[g]) not in hand or not (m.geom_contype[g] or m.geom_conaffinity[g]):continue
        v=local_mesh(m,d,g,cube)
        if v is None:unhandled.append(m.geom(g).name);continue
        vertical=max(float(v[:,1].min()+55/6),float(-27.5-v[:,1].max()))
        if vertical>0:
            row=dict(geom=m.geom(g).name,continuous_sweep_separation_proved=True,method='rotation-invariant vertical separation',minimum_separation_lower_bound_mm=vertical)
        else:
            row=dict(geom=m.geom(g).name,method='convex SAT angular samples plus intersample displacement bound',**sweep(v))
        rows.append(row)
    report=dict(status='whole_collision_hand_upper_layer_sweep_geometry',state=path.relative_to(OUT).as_posix(),collision_geoms=len(rows),unhandled_geoms=unhandled,all_existing_hand_geoms_clear=bool(rows and not unhandled and all(r['continuous_sweep_separation_proved'] for r in rows)),interfering=[r for r in rows if not r['continuous_sweep_separation_proved']],rows=rows,physical_clearance_verified=False,layer_dynamics_verified=False,devices_opened=False)
    (OUT/'whole_hand_upper_sweep_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))
if __name__=='__main__':main()
