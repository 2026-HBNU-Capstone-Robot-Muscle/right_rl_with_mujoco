import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
import numpy as np
import mujoco
from mujoco_playground import registry
from loaded_quarter_trial_plan_20261025 import ROOT,LoadedQuarterTrialPlan
from hand_motor_mapping import ticks_to_sim
from fingertip_cad_model_20261025 import build
from observed_book_model_policy import correct_model
from screen_loaded_quarter_trial_20261025 import cached_contacts
from book_grasp_trial_plan import RING_RELEASE
OUT=ROOT/'right_hand_rotation_20261024/loaded_quarter_trial_20261025'
p=LoadedQuarterTrialPlan(OUT/'fixture_v1/request.json');raw=p.source_position()
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build(env,OUT/'return_diagnostic.xml');correct_model(m,raw)
m.geom_contype[m.geom('cube').id]=0;m.geom_conaffinity[m.geom('cube').id]=0
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
inspect=cached_contacts(m,hand,{m.geom(n).id for n in ('support_book','support_glass','support_wood')})
result=[]
for delta in (0,4,-4):
    source=raw+delta;d=mujoco.MjData(m);worst={}
    for i,(phase,q) in enumerate([('initial',source)]+p.plan(source)):
        d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:]=[10,10,10,1,0,0,0];mujoco.mj_forward(m,d)
        _,pairs=inspect(d)
        for pair,depth in pairs.items():
            if depth>worst.get(pair,{}).get('depth_mm',0):worst[pair]=dict(depth_mm=depth,index=i,phase=phase,thumb_ticks=q[:4].tolist())
    result.append(dict(offset_all_ticks=delta,worst=worst));print(json.dumps(result[-1]),flush=True)
(OUT/'return_contact_diagnostic.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
