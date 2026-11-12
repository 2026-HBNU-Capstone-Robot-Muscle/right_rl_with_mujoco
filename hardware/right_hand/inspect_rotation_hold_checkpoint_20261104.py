"""Inspect saved model contact geometry; never opens physical devices."""
import json
from pathlib import Path
import numpy as np
import mujoco
from hand_motor_mapping import SIM_TO_MOTOR_IDS, TICK
from loaded_quarter_reentry_20261025 import limits

R=Path(__file__).resolve().parent
P=R/'right_hand_rotation_20261104/raised_cradle_160_yaw0/trace_nominal'
O=R/'right_hand_rotation_20261104/rotation_hold_session';O.mkdir(exist_ok=True)
m=mujoco.MjModel.from_binary_path(str(P/'closest_model.mjb'))
z=np.load(P/'closest_state.npz');d=mujoco.MjData(m)
for k in ('qpos','qvel','ctrl','act','qacc_warmstart'):getattr(d,k)[:]=z[k]
d.time=float(z['time']);mujoco.mj_forward(m,d)
cube=m.geom('cube').id;C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy()
contacts=[]
for i,c in enumerate(d.contact):
    a,b=int(c.geom1),int(c.geom2)
    if cube not in (a,b):continue
    other=b if a==cube else a
    f=np.zeros(6);mujoco.mj_contactForce(m,d,i,f)
    local=B.T@(c.pos-C)
    contacts.append(dict(geom=m.geom(other).name,distance_mm=float(c.dist*1000),normal_force_N=float(f[0]),cube_local_mm=(local*1000).tolist()))
lo,hi=limits();ids=np.asarray(SIM_TO_MOTOR_IDS)-1
result=dict(checkpoint=str(P.relative_to(R)),cube_center=C.tolist(),cube_rotation=B.tolist(),up=z['up'].tolist(),initial_cube_rotation=z['initial_cube_rotation'].tolist(),joints=[dict(name=m.joint(i).name,ID=int(SIM_TO_MOTOR_IDS[i]),actual=float(d.qpos[i]),command=float(d.ctrl[i]),command_ticks=float(d.ctrl[i]/TICK+2048),lower_ticks=float(lo[ids[i]]),upper_ticks=float(hi[ids[i]])) for i in range(16)],contacts=contacts,model_timestep=float(m.opt.timestep),devices_opened=False)
(O/'checkpoint_contact_review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,indent=2))
