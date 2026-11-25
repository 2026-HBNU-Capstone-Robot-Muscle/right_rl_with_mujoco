"""Explain the failed fresh-pose support case without changing any policy."""
import json,os
from pathlib import Path
import numpy as np
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco
from mujoco_playground import registry
from horizontal_quarter_plan_20261027 import HorizontalQuarterPlan,ROOT,PREFIX_PHASES,ENTRY,ROOTS
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from screen_loaded_repeat_20261026 import cached_contacts
from hand_motor_mapping import ticks_to_sim
from book_empty_preparation_plan import offsets
from book_support_contact_scope import derive_scope,allowed_initial_pairs

out=ROOT/'right_hand_rotation_20261027'/'edited_hand_204724'/'support_diagnosis';out.mkdir(exist_ok=True)
plan=HorizontalQuarterPlan(out.parent/'request.json');raw=plan.source_position()
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build(env,out/'scene.xml',book_projection_mm=-10);correct_model(m,raw);m.opt.timestep=.005
mask=m.geom_bodyid==m.body('cube').id;m.geom_contype[mask]=0;m.geom_conaffinity[mask]=0
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
inspect=cached_contacts(m,hand,support);allowed=allowed_initial_pairs(derive_scope(m));envelope={}
def put(d,q):
    d.qpos[:16]=ticks_to_sim(q);d.ctrl[:]=d.qpos[:16];d.qvel[:]=0;d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d)
for offset in offsets():
    d=mujoco.MjData(m);put(d,raw+offset);su,sp=inspect(d)
    for k,v in su.items():envelope[k]=max(envelope.get(k,0.),v)
m.actuator_gainprm[:,0]*=.5;m.actuator_biasprm[:,1]*=.5;d=mujoco.MjData(m);put(d,raw)
queue=[('initial_hold',raw,.5)]+[(p,q,.075) for p,q in plan.plan(raw) if p in PREFIX_PHASES]
records=[];maxinc=0.;schedule=0.;steps=0
for phase,q,duration in queue:
    d.ctrl[:]=ticks_to_sim(q);schedule+=duration;target=round(schedule/.005)
    for _ in range(target-steps):
        mujoco.mj_step(m,d)
        if phase not in ('initial_hold',ENTRY,ROOTS):continue
        su,sp=inspect(d)
        for pair,depth in su.items():
            limit=max(envelope.get(pair,0.),.15 if pair in allowed else 0.);excess=depth-limit
            if excess>maxinc+1e-9:
                maxinc=excess;records.append(dict(t=round(float(d.time),3),phase=phase,pair=pair,depth_mm=depth,
                    initial_envelope_mm=envelope.get(pair,0.),allowed_root_support=pair in allowed,excess_mm=excess))
    steps=target
report=dict(status='diagnostic_only',support_increase_events=records,maximum_excess_mm=maxinc,
    initial_envelope=envelope,hardware_execution_allowed=False,motor_writes=0)
(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(status=report['status'],maximum_excess_mm=maxinc,event_count=len(records),first=records[0] if records else None,last=records[-1] if records else None)),flush=True)
