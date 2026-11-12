"""List existing modeled contact and its change during EMPTY recovery."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim
from grasp_place_plan import ROOT,GraspPlacePlan

folder=ROOT/'prepared_cube_review_20261022/grasp_trial_v2_fixture'
plan=GraspPlacePlan(folder/'request.json');raw=plan.source_position()
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build_with_middle_thumb_hulls(env);d=mujoco.MjData(m)
cube=m.geom('cube').id;m.geom_contype[cube]=0;m.geom_conaffinity[cube]=0
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
def contacts(raw):
    d.qpos[:16]=ticks_to_sim(raw);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
    pairs={}
    for c in d.contact:
        a,b=int(c.geom1),int(c.geom2)
        if int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand and c.dist<0:
            key='|'.join(sorted((m.geom(a).name,m.geom(b).name)))
            pairs[key]=max(pairs.get(key,0),-float(c.dist)*1000)
    return pairs
initial=contacts(raw);previous=initial;new_pairs=set();increases={};trace=[]
for i,(phase,q) in enumerate(plan.plan(raw),1):
    pairs=contacts(q)
    for key,v in pairs.items():
        if key not in initial:new_pairs.add(key)
        increases[key]=max(increases.get(key,0),v-previous.get(key,0))
    if i%10==0 or phase!='stage_0_empty_recover' and previous:trace.append(dict(i=i,phase=phase,depths=pairs))
    previous=pairs
report=dict(initial_pairs_mm=initial,new_pairs=sorted(new_pairs),max_one_step_increase_mm=increases,
    nominal_initial_position=raw.tolist(),trace=trace,hardware_commands_sent=0)
(folder/'overlap_diagnostic.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2),flush=True)
