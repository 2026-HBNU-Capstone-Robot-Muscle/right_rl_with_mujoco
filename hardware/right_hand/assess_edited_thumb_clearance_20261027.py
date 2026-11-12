"""Compare small existing-route thumb advances; analysis only, no devices."""
import json,os
import numpy as np
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco
from mujoco_playground import registry
from horizontal_quarter_plan_20261027 import HorizontalQuarterPlan,ROOT,PREFIX_PHASES,ENTRY,ROOTS
from horizontal_thumb_support_clearance_20261027 import clear_thumb,THUMB_CLEAR
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from loaded_quarter_reentry_20261025 import validate_path
from screen_loaded_repeat_20261026 import cached_contacts
from hand_motor_mapping import ticks_to_sim
from book_empty_preparation_plan import offsets
from book_support_contact_scope import derive_scope,allowed_initial_pairs
out=ROOT/'right_hand_rotation_20261027'/'edited_hand_204724'/'thumb_clear_candidates';out.mkdir(exist_ok=True)
plan=HorizontalQuarterPlan(out.parent/'request.json');raw=plan.source_position()
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build(env,out/'scene.xml',book_projection_mm=-10);correct_model(m,raw);m.opt.timestep=.005
mask=m.geom_bodyid==m.body('cube').id;m.geom_contype[mask]=0;m.geom_conaffinity[mask]=0
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
inspect=cached_contacts(m,hand,support);allowed=allowed_initial_pairs(derive_scope(m))
def put(d,q):
    d.qpos[:16]=ticks_to_sim(q);d.ctrl[:]=d.qpos[:16];d.qvel[:]=0;d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d)
gains=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy();results=[]
for limit in (8,16,32):
    maxself=maxsupport=0.;envelope={}
    for offset in offsets():
        a=raw+offset;rows=clear_thumb(a,[(p,q) for p,q in plan.plan(a) if p in PREFIX_PHASES],limit)
        validate_path(a,rows,ENTRY);d=mujoco.MjData(m);put(d,a);su,sp=inspect(d)
        for pair,depth in su.items():envelope[pair]=max(envelope.get(pair,0.),depth)
        for phase,q in rows:
            put(d,q);su,sp=inspect(d);maxself=max(maxself,max(sp.values(),default=0.));maxsupport=max(maxsupport,max(su.values(),default=0.))
    dynamics=[]
    for scale in (.5,1.,1.5):
        m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias;m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale
        rows=clear_thumb(raw,[(p,q) for p,q in plan.plan(raw) if p in PREFIX_PHASES],limit);d=mujoco.MjData(m);put(d,raw)
        selfmax=supmax=increase=track=0.;schedule=0.;steps=0
        for phase,q,duration in [('initial_hold',raw,.5)]+[(p,q,.075) for p,q in rows]+[('final_hold',rows[-1][1],1.5)]:
            d.ctrl[:]=ticks_to_sim(q);schedule+=duration;target=round(schedule/.005)
            for _ in range(target-steps):
                mujoco.mj_step(m,d);su,sp=inspect(d);selfmax=max(selfmax,max(sp.values(),default=0.));supmax=max(supmax,max(su.values(),default=0.));track=max(track,float(max(abs(d.qpos[:16]-d.ctrl))))
                if phase in ('initial_hold',ENTRY,THUMB_CLEAR,ROOTS):
                    increase=max(increase,max((v-max(envelope.get(k,0.),.15 if k in allowed else 0.) for k,v in su.items()),default=0.))
            steps=target
        dynamics.append(dict(scale=scale,self_mm=selfmax,support_mm=supmax,initial_support_increase_mm=increase,tracking_rad=track))
    row=dict(advance_ticks=limit,max_static_self_mm=maxself,max_static_support_mm=maxsupport,dynamics=dynamics,
        passed=bool(maxself<=1e-6 and maxsupport<=.15 and all(v['self_mm']<=1e-6 and v['support_mm']<=.15 and v['initial_support_increase_mm']<=1e-6 and v['tracking_rad']<=.16 for v in dynamics)))
    results.append(row);print(json.dumps(row),flush=True)
(out/'review.json').write_text(json.dumps(dict(status='offline_candidate_comparison',cases=results,motor_writes=0),indent=2)+'\n',encoding='utf-8')
