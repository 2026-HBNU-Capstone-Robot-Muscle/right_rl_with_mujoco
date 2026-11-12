"""Test short thumb waypoints to avoid the index-base neighborhood collision."""
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
from screen_book_empty_preparation import contact_metrics

folder=ROOT/'right_hand_rotation_20261024/book20_current_review/preparation_fixture'
plan=BookPreparationPlan(folder/'request.json');raw=plan.source_position();opened,_=endpoints()
r=json.loads((folder/'screen_v1.json').read_bytes())
critical=np.array(max(r['static_screens'],key=lambda s:s['max_self_overlap_mm'])['offset_ticks'])
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build_book(env,folder/'thumb_detour_scene.xml',book_projection_mm=-5.,book_ylo_mm=-500.,book_yhi_mm=500.,support_projection_mm=50.)
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
d=mujoco.MjData(m)

def candidate(start,motor,delta):
    root=start.copy();root[ROOTS]=np.maximum(root[ROOTS],2472)
    detour=root.copy();detour[motor-1]+=delta
    rows=[];interpolate(rows,start,root,LIFT);interpolate(rows,root,detour,'thumb_clearance')
    nt=max(20,int(np.ceil(max(abs(opened[:4]-detour[:4]))/2)))
    nf=max(20,int(np.ceil(max(abs(opened[4:]-detour[4:]))/2)))
    for k in range(1,max(nt,nf)+1):
        q=detour.copy()
        q[:4]=np.rint(detour[:4]+(opened[:4]-detour[:4])*min(1.,k/nt)).astype(int)
        q[4:]=np.rint(detour[4:]+(opened[4:]-detour[4:])*min(1.,k/nf)).astype(int)
        rows.append(('prepare',q))
    return rows

def check(cases,motor,delta):
    maximum=after=0.;count=0
    for off in cases:
        rows=candidate(raw+off,motor,delta);count=max(count,len(rows))
        for phase,q in rows:
            d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
            sp,sm=contact_metrics(m,d,hand,support)
            maximum=max(maximum,sm)
            if phase!=LIFT:after=max(after,max(sp.values(),default=0.))
    return dict(max_self_overlap_mm=maximum,max_support_after_lift_mm=after,maximum_goals=count)

records=[];survivors=[]
for motor in (1,2,3,4):
    for delta in (-64,-32,32,64):
        metrics=check([np.zeros(16,dtype=int),critical],motor,delta)
        row=dict(motor=motor,detour_ticks=delta,**metrics);records.append(row)
        if metrics['max_self_overlap_mm']<=1e-6 and metrics['max_support_after_lift_mm']<=1e-6:survivors.append(row)
print('two-case survivors',json.dumps(survivors),flush=True)
verified=[]
for row in survivors:
    metrics=check(offsets(),row['motor'],row['detour_ticks'])
    result=dict(motor=row['motor'],detour_ticks=row['detour_ticks'],**metrics)
    verified.append(result);print('67-case detour',json.dumps(result),flush=True)
    if metrics['max_self_overlap_mm']<=1e-6 and metrics['max_support_after_lift_mm']<=1e-6:break
(folder/'thumb_detour_search.json').write_text(json.dumps(dict(two_case_candidates=records,
    neighborhood_cases=verified),indent=2),encoding='utf-8')
