"""Offline contact locations along the saved candidate, no motor interface."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from book_grasp_trial_plan import BookGraspPlan,ROOT
from book_support_model_20261024 import build_book
from hand_motor_mapping import ticks_to_sim

folder=ROOT/'right_hand_rotation_20261024/book20_current_review/grasp_route_fixture'
plan=BookGraspPlan(folder/'request.json');raw=plan.source_position()
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build_book(env,folder/'diagnostic_scene.xml',book_projection_mm=-5.,book_ylo_mm=-500.,book_yhi_mm=500.,support_projection_mm=50.)
m.geom_contype[m.geom('cube').id]=0;m.geom_conaffinity[m.geom('cube').id]=0
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
d=mujoco.MjData(m);first={};peaks={}
for i,(phase,q) in enumerate([('initial',raw)]+plan.plan(raw)):
    d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
    for c in d.contact:
        if c.dist>=-1e-9:continue
        a,b=int(c.geom1),int(c.geom2)
        if int(m.geom_bodyid[a]) not in hand or int(m.geom_bodyid[b]) not in hand:continue
        pair='|'.join(sorted((m.geom(a).name,m.geom(b).name)))
        item=dict(index=i,phase=phase,depth_mm=-float(c.dist)*1000,position_m=c.pos.tolist(),ticks=q.tolist())
        first.setdefault(pair,item)
        if pair not in peaks or peaks[pair]['depth_mm']<item['depth_mm']:peaks[pair]=item
r=dict(first=first,peak=peaks,hardware_commands_sent=0)
(folder/'contact_diagnosis.json').write_text(json.dumps(r,indent=2),encoding='utf-8')
print(json.dumps(r))
