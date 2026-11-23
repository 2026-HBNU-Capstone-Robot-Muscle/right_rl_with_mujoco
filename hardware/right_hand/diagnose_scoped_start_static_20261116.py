"""Find the first exact rejected static condition; no device access."""
import json
import numpy as np
import mujoco
from mujoco_playground import registry
from final_pose_temporary_support_20261116 import build
from final_pose_start_scope_20261116 import correct_model
from final_grip_pose_plan_20261116 import OUT,REQUEST,FinalGripPosePlan
from book_empty_preparation_plan import offsets
from book_support_contact_scope import derive_scope,allowed_initial_pairs
from screen_loaded_repeat_20261026 import cached_contacts
from hand_motor_mapping import ticks_to_sim
from new_pair_grasp_plan_20261027 import ENTRY,ROOTS
from calibrated_empty_ring_margin_20261028 import PHASE
p=FinalGripPosePlan(REQUEST);raw=p.source_position();cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build(env,OUT/'static_diagnostic.xml',book_projection_mm=-15);correct_model(m,raw)
mask=m.geom_bodyid==m.body('cube').id;m.geom_contype[mask]=0;m.geom_conaffinity[mask]=0
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
inspect=cached_contacts(m,hand,support);allowed=allowed_initial_pairs(derive_scope(m));d=mujoco.MjData(m)
def put(q):
    d.qpos[:16]=ticks_to_sim(q);d.ctrl[:]=d.qpos[:16];d.qvel[:]=0;d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d)
for off in offsets():
    a=raw+off;put(a);initial,sp=inspect(d);prior=initial
    bad=None
    if not set(initial)<=allowed:bad=dict(reason='unapproved_initial_pair',initial=initial,allowed=list(allowed))
    for phase,q in p.plan(a):
        put(q);su,sp=inspect(d)
        if phase in (ENTRY,PHASE,ROOTS):
            increases={k:v-initial.get(k,0.) for k,v in su.items() if v-initial.get(k,0.)>1e-6}
            steps={k:v-prior.get(k,0.) for k,v in su.items() if v-prior.get(k,0.)>1e-6}
            if increases or steps:bad=dict(reason='support_increase',phase=phase,increases=increases,steps=steps,initial=initial,goal=q.tolist())
            prior=su
        if bad:break
    if bad:
        print(json.dumps(dict(offset=off.tolist(),bad=bad)));break
else:print('No initial support failure found')
