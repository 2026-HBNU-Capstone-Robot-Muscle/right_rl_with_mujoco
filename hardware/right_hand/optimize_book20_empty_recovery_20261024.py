"""Nominal offline timing search, book footprint deliberately a hypothesis.

The roots lift first; simultaneous later joint interpolation is checked rather
than assumed. No production admission or external-contact limit is modified.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from book_support_model_20261024 import build_book
from hand_motor_mapping import ticks_to_sim
from grasp_place_plan import endpoints
from grasp_recovery_plan import interpolate

ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024/book20_current_review'
source=ROOT/'book_pose_readonly_20261024_054007_999816/result.json';blob=source.read_bytes();r=json.loads(blob)
assert r['status']=='photo_and_readings_saved' and r['pair_check']['maximum_drift_ticks']==0
raw=np.array([next(v['position_ticks_signed'] for v in r['snapshots'][0]['motors'] if v['id']==i) for i in range(1,17)])
opened,_=endpoints();root=raw.copy();root[[7,11,15]]=np.maximum(root[[7,11,15]],2472)
prefix=[];interpolate(prefix,raw,root,'roots')
nt=max(20,int(np.ceil(max(abs(opened[:4]-root[:4]))/2)))
nf=max(20,int(np.ceil(max(abs(opened[4:]-root[4:]))/2)))
def candidate(delay):
    rows=list(prefix)
    for k in range(1,max(nt,delay+nf)+1):
        q=root.copy();q[:4]=np.rint(root[:4]+(opened[:4]-root[:4])*min(1.,k/nt)).astype(int)
        q[4:]=np.rint(root[4:]+(opened[4:]-root[4:])*np.clip((k-delay)/nf,0,1)).astype(int)
        rows.append(('thumb_and_fingers',q))
    a=np.vstack([raw]+[q for _,q in rows]);assert np.max(abs(np.diff(a,axis=0)))<=2
    assert np.array_equal(a[-1],opened)
    return rows
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,manifest=build_book(env,OUT/'timing_scene.xml',book_projection_mm=0.,book_ylo_mm=-500.,book_yhi_mm=500.,support_projection_mm=50.)
d=mujoco.MjData(m);hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
def pose(q):
    angles=ticks_to_sim(q);d.qpos[:16]=angles;d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
    sp={};self_max=excess=0.
    for c in d.contact:
        if c.dist>=0:continue
        a,b=int(c.geom1),int(c.geom2);ha=int(m.geom_bodyid[a]) in hand;hb=int(m.geom_bodyid[b]) in hand
        key='|'.join(sorted((m.geom(a).name,m.geom(b).name)))
        if (a in support and hb) or (b in support and ha):sp[key]=max(sp.get(key,0.),-float(c.dist)*1000)
        if ha and hb:self_max=max(self_max,-float(c.dist)*1000)
    for i,j in enumerate(m.actuator_trnid[:,0]):
        lo,hi=m.jnt_range[j];clo,chi=m.actuator_ctrlrange[i]
        excess=max(excess,float(lo-angles[i]),float(angles[i]-hi),float(clo-angles[i]),float(angles[i]-chi))
    return sp,self_max,excess
results=[];selected=None;selected_rows=None
for delay in range(0,241,20):
    rows=candidate(delay);previous,initial_self,initial_range=pose(raw)
    max_self=initial_self;max_range=initial_range;new_deeper=0.;after_root=0.
    for phase,q in rows:
        sp,sm,excess=pose(q);max_self=max(max_self,sm);max_range=max(max_range,excess)
        new_deeper=max(new_deeper,max((v-previous.get(k,0.) for k,v in sp.items()),default=0.))
        if phase!='roots':after_root=max(after_root,max(sp.values(),default=0.))
        previous=sp
    row=dict(finger_delay_samples=delay,goals=len(rows),nominal_seconds=len(rows)*.05,
        delayed_cycle_budget_seconds=len(rows)*.06+2,max_self_model_overlap_mm=max_self,
        native_range_excess_rad=max_range,max_new_or_deeper_support_mm=new_deeper,
        max_support_after_root_phase_mm=after_root,
        nominal_kinematic_conditions_met=max_self<=1e-6 and max_range<=1e-6 and new_deeper<=1e-6 and after_root<=1e-6)
    results.append(row)
    if row['nominal_kinematic_conditions_met'] and selected is None:selected=row;selected_rows=rows
    print(json.dumps(row),flush=True)
record=dict(status='offline_empty_recovery_timing_search_complete',source_sha256=hashlib.sha256(blob).hexdigest(),
    book_thickness_mm_user_reported=20.,manifest=manifest,results=results,selected=selected,
    production_motor_code_changed=False,production_admission_changed=False,
    native_ranges_unchanged=True,hardware_execution_allowed=False,physical_calibration_verified=False,
    motor_commands_sent=0,limitations=['Book front 0mm and broad lateral slab are diagnostic hypotheses.',
        'Nominal static search only; existing support-contact overlap has not been physically calibrated or approved.',
        'No neighborhood or dynamic certificate, actual robot execution or cube motion is claimed.'])
(OUT/'timing_search.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
if selected_rows is not None:
    (OUT/'timing_selected_offline_goals.json').write_text(json.dumps(dict(hardware_execution_allowed=False,
        source_sha256=record['source_sha256'],raw_ticks=raw.tolist(),
        selected=selected,goals=[(p,q.tolist()) for p,q in selected_rows])),encoding='utf-8')
print('Selected nominal timing candidate:',json.dumps(selected),flush=True)
