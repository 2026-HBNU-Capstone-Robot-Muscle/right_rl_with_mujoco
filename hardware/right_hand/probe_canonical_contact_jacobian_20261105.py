"""Report local contact displacement per motor tick; static diagnosis only."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import json
import numpy as np
import mujoco
from dynamic_common_grip_search_20261105 import ROOT
from research_rotation_body_hold_20261104 import restore
from hand_motor_mapping import TICK, SIM_TO_MOTOR_IDS

p=ROOT/'full_placement_v2/front-10/g1_f0.3'
m=mujoco.MjModel.from_binary_path(str(p/'model.mjb'))
d=restore(m,np.load(p/'end_state.npz'))
cube=m.geom('cube').id
rows=[]
for name in ('mf_tip','th_tip'):
    gid=m.geom(name).id
    cs=[c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,gid}]
    for c in cs:
        if c.dist>.0004:continue
        outward=np.array(c.frame[:3])*(1 if int(c.geom1)==cube else -1)
        jp=np.zeros((3,m.nv));jr=jp.copy()
        mujoco.mj_jac(m,d,jp,jr,c.pos,int(m.geom_bodyid[gid]))
        values=-outward@jp[:,:16]*TICK*1000
        force=np.zeros(6);mujoco.mj_contactForce(m,d,list(d.contact).index(c),force)
        rows.append(dict(geom=name,contact_force_N=float(force[0]),
                         inward_displacement_mm_per_positive_tick={str(mid):float(v) for mid,v in zip(SIM_TO_MOTOR_IDS,values) if abs(v)>.00001}))
report=dict(status='static_contact_jacobian_only',cases=rows,not_a_free_cube_motion_or_force_proof=True,devices_opened=False)
(ROOT/'contact_squeeze_jacobian.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report),flush=True)
