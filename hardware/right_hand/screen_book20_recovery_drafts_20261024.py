"""Offline staged EMPTY recovery drafts from the saved current book pose.

No historical admission gate is changed. These drafts are not motor programs,
and unmeasured book footprints cannot become deployment certificates.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from book_support_model_20261024 import build_book,MEASUREMENTS
from hand_motor_mapping import ticks_to_sim
from grasp_place_plan import endpoints
from grasp_recovery_plan import interpolate

ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024/book20_current_review'
source_path=ROOT/'book_pose_readonly_20261024_054007_999816/result.json'
blob=source_path.read_bytes();source=json.loads(blob)
assert source['status']=='photo_and_readings_saved' and source['pair_check']['maximum_drift_ticks']==0
raw=np.array([next(v['position_ticks_signed'] for v in source['snapshots'][0]['motors'] if v['id']==i) for i in range(1,17)])
opened,_=endpoints();roots=np.array([7,11,15])

def drafts():
    root=raw.copy();root[roots]=np.maximum(root[roots],2472)
    thumb=root.copy();thumb[:4]=opened[:4]
    # Keep all selected variants explicit; do not optimize/fix joint offsets.
    variants={
        'roots_then_thumb_then_fingers':[('roots',root),('thumb',thumb),('fingers',opened)],
        'roots_thumb_flexion_then_remaining':[],
        'roots_thumb_root_then_remaining':[]}
    flex=root.copy();flex[[0,1]]=opened[[0,1]]
    variants['roots_thumb_flexion_then_remaining']=[('roots',root),('thumb_flexion',flex),('thumb',thumb),('fingers',opened)]
    cmc=root.copy();cmc[3]=opened[3]
    variants['roots_thumb_root_then_remaining']=[('roots',root),('thumb_root',cmc),('thumb',thumb),('fingers',opened)]
    out={}
    for name,waypoints in variants.items():
        rows=[];a=raw.copy()
        for phase,b in waypoints:a=interpolate(rows,a,b,phase)
        assert max(np.max(abs(b-a)) for a,b in zip([raw]+[q for _,q in rows[:-1]],[q for _,q in rows]))<=2
        out[name]=rows
    return out

def clip(poly,axis,bound,sign):
    if not len(poly):return []
    out=[];previous=poly[-1];before=sign*(previous[axis]-bound)<=1e-8
    for point in poly:
        inside=sign*(point[axis]-bound)<=1e-8
        if inside!=before:
            t=(bound-previous[axis])/(point[axis]-previous[axis]);out.append(previous+t*(point-previous))
        if inside:out.append(point)
        previous=point;before=inside
    return out

cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
variants=drafts();records=[];audits=[]
for front in (-20.,0.,20.):
    model,manifest=build_book(env,OUT/'draft_scene.xml',book_projection_mm=front,book_yhi_mm=65.,support_projection_mm=50.)
    data=mujoco.MjData(model);palm=model.body('palm').id;hand={palm}
    for body in range(model.nbody):
        if int(model.body_parentid[body]) in hand:hand.add(body)
    support={model.geom(n).id for n in ('support_glass','support_wood','support_book')}
    def put(q):
        angles=ticks_to_sim(q);data.qpos[:16]=angles;data.qpos[-7:-4]=[0,0,2];data.qpos[-4:]=[1,0,0,0]
        mujoco.mj_forward(model,data)
        pairs={};self_pairs={}
        for c in data.contact:
            if c.dist>=0:continue
            a,b=int(c.geom1),int(c.geom2);ha=int(model.geom_bodyid[a]) in hand;hb=int(model.geom_bodyid[b]) in hand
            key='|'.join(sorted((model.geom(a).name,model.geom(b).name)))
            if (a in support and hb) or (b in support and ha):pairs[key]=max(pairs.get(key,0.),-float(c.dist)*1000)
            if ha and hb:self_pairs[key]=max(self_pairs.get(key,0.),-float(c.dist)*1000)
        excess=0.
        for i,j in enumerate(model.actuator_trnid[:,0]):
            lo,hi=model.jnt_range[j];clo,chi=model.actuator_ctrlrange[i]
            excess=max(excess,float(lo-angles[i]),float(angles[i]-hi),float(clo-angles[i]),float(angles[i]-chi))
        return pairs,self_pairs,excess
    for name,rows in variants.items():
        previous,_,initial_excess=put(raw);initial=dict(previous);phases={};new_or_deeper=0.
        self_max=0.;range_max=initial_excess;clear_at=None
        for number,(phase,q) in enumerate(rows,1):
            pairs,self_pairs,excess=put(q)
            self_max=max(self_max,max(self_pairs.values(),default=0.));range_max=max(range_max,excess)
            new_or_deeper=max(new_or_deeper,max((v-previous.get(k,0.) for k,v in pairs.items()),default=0.))
            if clear_at is None and not pairs:clear_at=number
            row=phases.setdefault(phase,dict(max_support_model_overlap_mm=0.,max_self_model_overlap_mm=0.))
            row['max_support_model_overlap_mm']=max(row['max_support_model_overlap_mm'],max(pairs.values(),default=0.))
            row['max_self_model_overlap_mm']=max(row['max_self_model_overlap_mm'],max(self_pairs.values(),default=0.))
            previous=pairs
        records.append(dict(book_front_mm_hypothesis=front,name=name,goals=len(rows),nominal_seconds=len(rows)*.05,
            initial_support_pairs_mm=initial,phases=phases,native_range_excess_rad=range_max,
            max_new_or_deeper_support_mm=new_or_deeper,max_self_model_overlap_mm=self_max,
            first_support_clear_goal=clear_at,final_goal=rows[-1][1].tolist(),hardware_execution_allowed=False))
    # Check actual CAD surfaces at the measured start, independently of the
    # mesh convex hull. Surface crossing does not measure real-world contact.
    put(raw);rotation=data.xmat[palm].reshape(3,3);origin=data.xpos[palm]
    boxes=dict(manifest['bounds_palm_mm'],support_book=manifest['book_bounds_palm_mm'])
    for name in ('if_bs_visual_support_proxy','mf_bs_visual_support_proxy','rf_bs_visual_support_proxy'):
        g=model.geom(name).id;mid=int(model.geom_dataid[g]);va=int(model.mesh_vertadr[mid]);vn=int(model.mesh_vertnum[mid])
        fa=int(model.mesh_faceadr[mid]);fn=int(model.mesh_facenum[mid])
        points=model.mesh_vert[va:va+vn]@data.geom_xmat[g].reshape(3,3).T+data.geom_xpos[g]
        points=(points-origin)@rotation*1000;triangles=points[model.mesh_face[fa:fa+fn]]
        for support_name,(lower,upper) in boxes.items():
            lower,upper=np.array(lower),np.array(upper)
            mask=np.all(triangles.max(1)>=lower,axis=1)&np.all(triangles.min(1)<=upper,axis=1)
            count=0
            for tri in triangles[mask]:
                poly=list(tri)
                for axis in range(3):
                    poly=clip(poly,axis,lower[axis],-1);poly=clip(poly,axis,upper[axis],1)
                    if not len(poly):break
                count+=len(poly)>=3
            audits.append(dict(book_front_mm_hypothesis=front,geom=name,support=support_name,
                intersecting_CAD_surface_triangles=int(count),broad_phase_triangles=int(mask.sum())))
    print(f'Book front hypothesis {front:g}: three drafts and native CAD surface audit complete.',flush=True)

report=dict(status='offline_empty_recovery_drafts_reviewed_not_approved',source_sha256=hashlib.sha256(blob).hexdigest(),
    book_measurement_sha256=hashlib.sha256(MEASUREMENTS.read_bytes()).hexdigest(),
    book_thickness_mm_user_reported=20.,raw_ticks=raw.tolist(),records=records,CAD_surface_audit=audits,
    current_production_plan_admission_unchanged=True,native_ranges_unchanged=True,
    motor_commands_sent=0,real_devices_opened=False,hardware_execution_allowed=False,
    limitations=['Book front and width remain unmeasured hypotheses.',
        'Only nominal kinematics are checked. No neighborhood or dynamic execution certificate is issued.',
        'Initial model overlaps and CAD crossings do not establish actual physical contact or its cause.',
        'These candidates are not loaded by any motor launcher.'])
(OUT/'empty_recovery_draft_review.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
(OUT/'empty_recovery_draft_goals.json').write_text(json.dumps(dict(hardware_execution_allowed=False,
    variants={name:[(phase,q.tolist()) for phase,q in rows] for name,rows in variants.items()})),encoding='utf-8')
print(json.dumps(dict(status=report['status'],drafts=[{k:r[k] for k in ('book_front_mm_hypothesis','name','goals',
    'native_range_excess_rad','max_self_model_overlap_mm','max_new_or_deeper_support_mm','first_support_clear_goal')} for r in records],
    CAD_surface_crossings=[a for a in audits if a['intersecting_CAD_surface_triangles']],
    hardware_execution_allowed=False),indent=2),flush=True)
