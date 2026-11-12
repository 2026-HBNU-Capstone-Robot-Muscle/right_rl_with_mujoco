"""Inspect the worst actual failed neighborhood, without device access."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from book_support_model_20261024 import build_book
from book_empty_preparation_plan import BookPreparationPlan,LIFT,ROOT
from hand_motor_mapping import ticks_to_sim
from screen_book_empty_preparation import contact_metrics

folder=ROOT/'right_hand_rotation_20261024/book20_current_review/preparation_fixture'
r=json.loads((folder/'screen_v1.json').read_bytes())
worst=max(r['static_screens'],key=lambda s:s['max_self_overlap_mm'])
print('worst static',json.dumps(worst),flush=True)
plan=BookPreparationPlan(folder/'request.json');raw=plan.source_position()+np.array(worst['offset_ticks'])
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build_book(env,folder/'diagnosis_scene.xml',book_projection_mm=worst['book_front_mm'],
    book_ylo_mm=-500.,book_yhi_mm=500.,support_projection_mm=50.)
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
d=mujoco.MjData(m);found=[]
for index,(phase,q) in enumerate([('initial',raw)]+plan.plan(raw)):
    d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
    pairs={}
    for c in d.contact:
        a,b=int(c.geom1),int(c.geom2)
        if c.dist<0 and int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand:
            key='|'.join(sorted((m.geom(a).name,m.geom(b).name)));pairs[key]=max(pairs.get(key,0.),-float(c.dist)*1000)
    if pairs:found.append(dict(index=index,phase=phase,pairs=pairs,goal=q.tolist()))
print('self contact samples',len(found),flush=True)
if found:
    print('first',json.dumps(found[0]),flush=True)
    print('worst',json.dumps(max(found,key=lambda s:max(s['pairs'].values()))),flush=True)
    print('last',json.dumps(found[-1]),flush=True)
(folder/'failed_neighborhood_diagnosis.json').write_text(json.dumps(dict(worst_static=worst,
    self_contact_samples=found),indent=2),encoding='utf-8')
