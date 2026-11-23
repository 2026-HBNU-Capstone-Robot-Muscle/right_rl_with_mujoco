"""Offline only: locate the contact, without changing any stop criterion."""
import json
import numpy as np
import mujoco
from final_grip_pose_plan_20261116 import ROOT,OUT,REQUEST,FinalGripPosePlan
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_model_policy_20261025 import correct_model
from screen_loaded_repeat_20261026 import cached_contacts

p=FinalGripPosePlan(REQUEST);raw=p.source_position();rows=p.plan(raw)
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build(env,OUT/'diagnostic_scene.xml',book_projection_mm=-10);correct_model(m,raw);m.opt.timestep=.005
mask=m.geom_bodyid==m.body('cube').id;m.geom_contype[mask]=0;m.geom_conaffinity[mask]=0
m.actuator_gainprm[:,0]*=.5;m.actuator_biasprm[:,1]*=.5
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
inspect=cached_contacts(m,hand,support)
d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(raw);d.ctrl[:]=d.qpos[:16];d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d)
worst={};scheduled=0.;steps=0
for i,(phase,q,duration) in enumerate([('initial_hold',raw,.5)]+[(p,q,.075) for p,q in rows]+[('final_hold',rows[-1][1],1.5)]):
    d.ctrl[:]=ticks_to_sim(q);scheduled+=duration;target=round(scheduled/.005)
    for _ in range(target-steps):
        mujoco.mj_step(m,d);_,sp=inspect(d)
        for pair,depth in sp.items():
            if depth>worst.get(pair,{}).get('mm',0):
                actual=np.empty(16);actual[np.asarray(SIM_TO_MOTOR_IDS)-1]=d.qpos[:16]/TICK+2048
                worst[pair]=dict(mm=depth,phase=phase,goal_index=i,command=q.tolist(),actual_ticks=actual.tolist())
    steps=target
result=dict(worst=worst,devices_opened=False)
(OUT/'contact_diagnostic.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
