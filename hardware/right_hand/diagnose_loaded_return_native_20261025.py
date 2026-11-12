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
p=LoadedQuarterTrialPlan(OUT/'fixture_v2/request.json');raw=p.source_position();rows=p.plan(raw)
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build(env,OUT/'return_native_diagnostic.xml');correct_model(m,raw)
m.geom_contype[m.geom('cube').id]=0;m.geom_conaffinity[m.geom('cube').id]=0
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
inspect=cached_contacts(m,hand,{m.geom(n).id for n in ('support_book','support_glass','support_wood')})
m.actuator_gainprm[:,0]*=.5;m.actuator_biasprm[:,1]*=.5
d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(raw);d.qpos[-7:]=[10,10,10,1,0,0,0];mujoco.mj_forward(m,d)
allrows=[('initial_hold',raw)]*10+rows;worst={};time=0.
for i,(phase,rawgoal) in enumerate(allrows):
    d.ctrl[:]=ticks_to_sim(rawgoal)
    for _ in range(round(.05/m.opt.timestep)):
        mujoco.mj_step(m,d);time+=m.opt.timestep;_,pairs=inspect(d)
        if phase in ('initial_hold',RING_RELEASE):continue
        for pair,depth in pairs.items():
            if depth>worst.get(pair,{}).get('depth_mm',0):worst[pair]=dict(depth_mm=depth,index=i,time=time,phase=phase,goal_ticks=rawgoal.tolist())
print(json.dumps(worst),flush=True)
(OUT/'return_native_contact_diagnostic.json').write_text(json.dumps(worst,indent=2),encoding='utf-8')
