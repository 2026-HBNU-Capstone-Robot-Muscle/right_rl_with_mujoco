import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,numpy as np,mujoco
from mujoco_playground import registry
from loaded_quarter_trial_plan_20261025 import ROOT,LoadedQuarterTrialPlan
from hand_motor_mapping import ticks_to_sim
from fingertip_cad_model_20261025 import build
from observed_book_model_policy import correct_model
from screen_loaded_quarter_trial_20261025 import cached_contacts
OUT=ROOT/'right_hand_rotation_20261024/loaded_quarter_trial_20261025/fixture_v4'
p=LoadedQuarterTrialPlan(OUT/'request.json');raw=p.source_position()
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build(env,OUT/'support_witness.xml',book_projection_mm=-5);correct_model(m,raw)
m.geom_contype[m.geom('cube').id]=0;m.geom_conaffinity[m.geom('cube').id]=0
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
inspect=cached_contacts(m,hand,{m.geom(n).id for n in ('support_book','support_glass','support_wood')})
d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(raw);d.qpos[-7:]=[10,10,10,1,0,0,0];mujoco.mj_forward(m,d)
initial,_=inspect(d);worst={};last={};t=0.
m.actuator_gainprm[:,0]*=.5;m.actuator_biasprm[:,1]*=.5
for i,(phase,goal) in enumerate([('initial_hold',raw)]*10+p.plan(raw)[:280]):
    d.ctrl[:]=ticks_to_sim(goal)
    for _ in range(round(.05/m.opt.timestep)):
        mujoco.mj_step(m,d);t+=m.opt.timestep;pairs,_=inspect(d)
        for pair,depth in pairs.items():
            last[pair]=t
            if depth>worst.get(pair,{}).get('depth_mm',0):worst[pair]=dict(depth_mm=depth,time=t,phase=phase,goal=goal.tolist())
result=dict(initial_support=initial,maximums=worst,last_contact_seconds=last)
print(json.dumps(result),flush=True);(OUT/'support_witness.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
