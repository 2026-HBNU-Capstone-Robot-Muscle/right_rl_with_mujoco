"""Offline timing revision after an actual neighborhood collision failure."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
import numpy as np
import mujoco
from mujoco_playground import registry
from book_support_model_20261024 import build_book
from book_empty_preparation_plan import BookPreparationPlan,ROOT,ROOTS,LIFT,offsets
from hand_motor_mapping import ticks_to_sim
from grasp_place_plan import endpoints
from grasp_recovery_plan import interpolate

folder=ROOT/'right_hand_rotation_20261024/book20_current_review/preparation_fixture'
plan=BookPreparationPlan(folder/'request.json');raw=plan.source_position();opened,_=endpoints()
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build_book(env,folder/'timing_revision_scene.xml',book_projection_mm=-10.,book_ylo_mm=-500.,book_yhi_mm=500.,support_projection_mm=50.)
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
d=mujoco.MjData(m)
records=[]
for shift in (-60,-40,-20,20,40):
    maximum=0.;bad_case=None;nominal_count=0
    for off in offsets():
        start=raw+off;root=start.copy();root[ROOTS]=np.maximum(root[ROOTS],2472)
        nt=max(20,int(np.ceil(max(abs(opened[:4]-root[:4]))/2)))
        nf=max(20,int(np.ceil(max(abs(opened[4:]-root[4:]))/2)))
        td=max(0,-shift);fd=max(0,shift);count=max(nt+td,nf+fd)
        nominal_count=max(nominal_count,count+int(np.ceil(max(abs(root-start))/2)))
        for k in range(1,count+1):
            q=root.copy()
            q[:4]=np.rint(root[:4]+(opened[:4]-root[:4])*np.clip((k-td)/nt,0,1)).astype(int)
            q[4:]=np.rint(root[4:]+(opened[4:]-root[4:])*np.clip((k-fd)/nf,0,1)).astype(int)
            d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
            for c in d.contact:
                a,b=int(c.geom1),int(c.geom2)
                if c.dist<0 and int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand:
                    depth=-float(c.dist)*1000
                    if depth>maximum:
                        maximum=depth;bad_case=dict(offset=off.tolist(),sample=k,
                            pair='|'.join(sorted((m.geom(a).name,m.geom(b).name))))
    row=dict(finger_delay_relative_thumb_samples=shift,max_self_overlap_mm=maximum,
        maximum_goals_across_neighborhood=nominal_count,worst=bad_case)
    records.append(row);print(json.dumps(row),flush=True)
(folder/'timing_neighborhood_revision.json').write_text(json.dumps(records,indent=2),encoding='utf-8')
