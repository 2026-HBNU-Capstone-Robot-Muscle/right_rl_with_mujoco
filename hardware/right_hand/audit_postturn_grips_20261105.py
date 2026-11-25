"""Existing CAD geometry versus upper-layer sweep; saved-model analysis only."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import json,time
from pathlib import Path
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import restore,inspect_for
from audit_upper_layer_clearance_20261104 import local_mesh,box_gap
from audit_upper_layer_sweep_20261104 import sweep
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/preintegration/body_grip'

def audit(m,z):
    d=restore(m,z);cube=m.geom('cube').id;hand,_,_=inspect_for(m);rows=[]
    for g in range(m.ngeom):
        if int(m.geom_bodyid[g]) not in hand or not(m.geom_contype[g] or m.geom_conaffinity[g]):continue
        v=local_mesh(m,d,g,cube)
        if v is None:rows.append(dict(geom=m.geom(g).name,continuous_sweep_separation_proved=False,method='unhandled'));continue
        vertical=max(float(v[:,1].min()+55/6),float(-27.5-v[:,1].max()))
        conservative=box_gap(v,np.array([0.,-55/3,0]),np.array([27.5*np.sqrt(2),55/6,27.5*np.sqrt(2)]))
        if vertical>0:row=dict(geom=m.geom(g).name,continuous_sweep_separation_proved=True,method='vertical separation',gap_mm=vertical)
        elif conservative>0:row=dict(geom=m.geom(g).name,continuous_sweep_separation_proved=True,method='circumscribed square separation',gap_mm=conservative)
        else:row=dict(geom=m.geom(g).name,method='angular SAT plus between-sample bound',**sweep(v))
        rows.append(row)
    return dict(all_hand_geoms_clear=all(r['continuous_sweep_separation_proved'] for r in rows),geoms=len(rows),interfering=[r for r in rows if not r['continuous_sweep_separation_proved']],rows=rows)

def main():
    m=mujoco.MjModel.from_binary_path(str(O.parent/'reference_end/end_model.mjb'));results=[]
    for name in ('lower_m12_t12_n10','lower_m16_t12_n10','lower_m20_t16_n10','lower_m24_t16_n10','lower_m24_t20_n10'):
        p=O/name/'end_state.npz'
        if not p.exists():continue
        row=dict(name=name,**audit(m,np.load(p)));results.append(row);print(json.dumps({k:v for k,v in row.items() if k!='rows'}),flush=True)
    (O/'whole_hand_sweep.json').write_text(json.dumps(dict(status='offline_upper_layer_clearance',cases=results,physical_layer_turn_verified=False,hardware_execution_allowed=False,devices_opened=False),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
