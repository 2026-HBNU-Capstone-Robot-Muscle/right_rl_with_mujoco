import os
os.environ['JAX_PLATFORMS']='cpu'
import json
from pathlib import Path
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import restore
from audit_upper_layer_clearance_20261104 import local_mesh
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/preintegration/reference_end'
m=mujoco.MjModel.from_binary_path(str(O/'end_model.mjb'));d=restore(m,np.load(O/'end_state.npz'));cube=m.geom('cube').id;B=d.geom_xmat[cube].reshape(3,3);C=d.geom_xpos[cube]
rows=[]
for c in d.contact:
    if cube not in (int(c.geom1),int(c.geom2)):continue
    g=int(c.geom2) if int(c.geom1)==cube else int(c.geom1)
    rows.append(dict(geom=m.geom(g).name,cube_local_position_mm=((c.pos-C)@B*1000).tolist(),cube_local_normal=(B.T@c.frame[:3]).tolist()))
print('CONTACTS',json.dumps(rows))
for name in ('mf_tip','mf_ds_collision_1','mf_md_collision_5','th_tip'):
    v=local_mesh(m,d,m.geom(name).id,cube);print(name,json.dumps(dict(min=v.min(0).tolist(),max=v.max(0).tolist())))
