"""Native geometry of the freshly measured EMPTY pose; no control interface."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from book_support_model_20261024 import build_book
from hand_motor_mapping import ticks_to_sim
root=Path(__file__).resolve().parent
folder=root/'book_settled_off_readonly_20261024_174424_563949'
request=json.loads((folder/'request.json').read_bytes());raw=np.array(request['off_state']['position'])
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build_book(env,folder/'diagnostic_scene.xml',book_projection_mm=-10.,book_ylo_mm=-500.,book_yhi_mm=500.,support_projection_mm=50.)
m.geom_contype[m.geom('cube').id]=0;m.geom_conaffinity[m.geom('cube').id]=0
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
d=mujoco.MjData(m);rows=[]
for delta in (0,50,100):
    q=raw.copy();q[3]+=delta
    d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
    pairs={}
    for c in d.contact:
        if c.dist>=0:continue
        a,b=int(c.geom1),int(c.geom2)
        if int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand:
            name='|'.join(sorted((m.geom(a).name,m.geom(b).name)));pairs[name]=max(pairs.get(name,0.),-float(c.dist)*1000)
    local=(d.site('th_tip').xpos-d.body('palm').xpos)@d.body('palm').xmat.reshape(3,3)
    rows.append(dict(ID4_ticks=int(q[3]),delta_degrees=delta*360/4096,thumb_tip_palm_mm=(local*1000).tolist(),self_contacts_mm=pairs))
j=int(m.actuator_trnid[12,0]);result=dict(thumb_base_joint=m.joint(j).name,axis=m.jnt_axis[j].tolist(),native_range_rad=m.jnt_range[j].tolist(),rows=rows,hardware_commands_sent=0)
(folder/'geometry_review.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result))
