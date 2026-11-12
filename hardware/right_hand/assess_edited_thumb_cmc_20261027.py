"""Compare an early single-joint thumb lift; no hardware or policy changes."""
import json,os
import numpy as np
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco
from mujoco_playground import registry
from horizontal_quarter_plan_20261027 import HorizontalQuarterPlan,ROOT,PREFIX_PHASES,ENTRY,ROOTS,OPEN
from fresh_empty_grasp_plan_20261027 import interpolate
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from screen_loaded_repeat_20261026 import cached_contacts
from hand_motor_mapping import ticks_to_sim
from loaded_quarter_reentry_20261025 import validate_path
out=ROOT/'right_hand_rotation_20261027'/'edited_hand_204724'/'cmc_clear_candidates';out.mkdir(exist_ok=True)
plan=HorizontalQuarterPlan(out.parent/'request.json');raw=plan.source_position()
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build(env,out/'scene.xml',book_projection_mm=-10);correct_model(m,raw);m.opt.timestep=.005
mask=m.geom_bodyid==m.body('cube').id;m.geom_contype[mask]=0;m.geom_conaffinity[mask]=0
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
inspect=cached_contacts(m,hand,support)
def put(d,q):
    d.qpos[:16]=ticks_to_sim(q);d.ctrl[:]=d.qpos[:16];d.qvel[:]=0;d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d)
def route(a,delta):
    original=[(p,q) for p,q in plan.plan(a) if p in PREFIX_PHASES];first=next(i for i,(p,q) in enumerate(original) if p==ROOTS)
    start=(original[first-1][1] if first else a).copy();end=start.copy();end[3]+=delta
    rows=original[:first]+[('thumb_clear',q) for q in interpolate(start,end)]
    for phase,q in original[first:]:
        if phase!=ROOTS:break
        value=q.copy();value[3]=end[3];rows.append((phase,value))
    opened=[q for p,q in original if p==OPEN][-1]
    rows.extend((OPEN,q) for q in interpolate(rows[-1][1],opened));validate_path(a,rows,ENTRY);return rows
gains=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy();results=[]
for delta in (4,8,16,-4):
    rows=route(raw,delta);d=mujoco.MjData(m);put(d,raw);staticself=staticsup=0
    for phase,q in rows:
        put(d,q);su,sp=inspect(d);staticself=max(staticself,max(sp.values(),default=0.));staticsup=max(staticsup,max(su.values(),default=0.))
    m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias;m.actuator_gainprm[:,0]*=.5;m.actuator_biasprm[:,1]*=.5
    d=mujoco.MjData(m);put(d,raw);selfmax=supmax=early=track=0.;steps=0;schedule=0.;worst={}
    for phase,q,duration in [('initial_hold',raw,.5)]+[(p,q,.075) for p,q in rows]+[('final_hold',rows[-1][1],1.5)]:
        d.ctrl[:]=ticks_to_sim(q);schedule+=duration;target=round(schedule/.005)
        for _ in range(target-steps):
            mujoco.mj_step(m,d);su,sp=inspect(d);selfmax=max(selfmax,max(sp.values(),default=0.));supmax=max(supmax,max(su.values(),default=0.));track=max(track,float(max(abs(d.qpos[:16]-d.ctrl))))
            if phase in ('initial_hold',ENTRY,ROOTS,'thumb_clear'):
                early=max(early,max(su.values(),default=0.))
                for pair,depth in su.items():worst[pair]=max(worst.get(pair,0.),depth)
        steps=target
    row=dict(delta_id4_ticks=delta,static_self_mm=staticself,static_support_mm=staticsup,dynamic_self_mm=selfmax,dynamic_support_mm=supmax,early_support_mm=early,early_pairs=worst,tracking_rad=track)
    results.append(row);print(json.dumps(row),flush=True)
(out/'review.json').write_text(json.dumps(dict(status='offline_candidates_only',cases=results),indent=2)+'\n',encoding='utf-8')
