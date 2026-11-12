"""Compare small index-root changes; preserve contact thresholds."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fresh_empty_grasp_plan_20261027 import FreshEmptyGraspPlan,REQUEST,ROOT,RING_PHASE,ROOT_PHASE
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from hand_motor_mapping import ticks_to_sim
from screen_loaded_repeat_20261026 import cached_contacts
p=FreshEmptyGraspPlan(REQUEST);raw=p.source_position();rows=p.plan(raw);out=REQUEST.parent/'support_margin_study';out.mkdir(exist_ok=True);cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env,out/'scene.xml',book_projection_mm=-5.);correct_model(m,raw);hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support);results=[]
for minimum in (1965,1984,2000,2016,2048):
    d=mujoco.MjData(m);spmax=supmax=0.;worst=None
    for i,(phase,goal) in enumerate(rows):
        if phase in (RING_PHASE,ROOT_PHASE):continue
        q=goal.copy();q[7]=max(int(q[7]),minimum);d.qpos[:16]=ticks_to_sim(q);mujoco.mj_forward(m,d);su,sp=inspect(d);spmax=max(spmax,max(sp.values(),default=0.))
        if max(su.values(),default=0.)>supmax:supmax=max(su.values());worst=dict(sample=i,phase=phase,pairs=su,goal=q.tolist())
    results.append(dict(index_root_minimum_ticks=minimum,max_self_mm=spmax,max_support_mm=supmax,worst=worst))
r=dict(status='index_root_clearance_adjustment_diagnostic',cases=results,hardware_execution_allowed=False,real_devices_opened=False);(out/'review.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');print(json.dumps(r,indent=2),flush=True)
