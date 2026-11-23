"""Inspect geometry only; no devices and no collision removal."""
import json
import numpy as np
import mujoco
from mujoco_playground import registry
from final_pose_temporary_support_20261116 import build
from final_grip_pose_plan_20261116 import OUT,REQUEST,FinalGripPosePlan
from final_pose_start_scope_20261116 import correct_model
from hand_motor_mapping import ticks_to_sim
p=FinalGripPosePlan(REQUEST);raw=p.source_position()
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build(env,OUT/'temporary_support_inspect.xml',book_projection_mm=-10);correct_model(m,raw)
d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(raw);d.ctrl[:]=d.qpos[:16];d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d)
names=['support_book','support_glass','support_wood','th_ds_collision_1']
names += [m.geom(i).name for i in range(m.ngeom) if m.geom_bodyid[i]==m.body('palm').id]
rows=[]
for name in names:
    if not name:continue
    g=m.geom(name).id
    item=dict(name=name,pos_mm=(d.geom_xpos[g]*1000).tolist(),size_mm=(m.geom_size[g]*1000).tolist(),type=int(m.geom_type[g]))
    mesh=m.geom_dataid[g]
    if mesh>=0:
        v=m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh]+m.mesh_vertnum[mesh]]
        v=v@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
        item['world_bounds_mm']=[(v.min(0)*1000).tolist(),(v.max(0)*1000).tolist()]
    rows.append(item)
print(json.dumps(rows))
from screen_loaded_repeat_20261026 import cached_contacts
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
inspect=cached_contacts(m,hand,support)
print('INITIAL_SUPPORT_AND_SELF',json.dumps(inspect(d)))
