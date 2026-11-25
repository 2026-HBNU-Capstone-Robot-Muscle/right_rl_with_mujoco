"""Inspect fresh post-book encoders with measured 20mm height; no devices."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from book_support_model_20261024 import build_book,MEASUREMENTS
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS
from grasp_place_plan import endpoints
from grasped_yaw_plan import GraspedYawPlan

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'book_pose_readonly_20261024_054007_999816'
OUT=ROOT/'right_hand_rotation_20261024'/'book20_current_review';OUT.mkdir(parents=True,exist_ok=True)
blob=(FOLDER/'result.json').read_bytes();source=json.loads(blob)
assert source['status']=='photo_and_readings_saved' and source['pair_check']['maximum_drift_ticks']<=3
assert source['servo_write_commands_sent']==0 and source['torque_changes_requested']==0
motors=source['snapshots'][0]['motors'];by_id={v['id']:v for v in motors}
assert sorted(by_id)==list(range(1,17))
assert all(v['torque_enabled']==v['torque_enabled_end']==v['hardware_error']==v['bus_watchdog_raw']==0 for v in motors)
raw=np.array([by_id[i]['position_ticks_signed'] for i in range(1,17)])
state=dict(position=raw.tolist(),torque=[by_id[i]['torque_enabled'] for i in range(1,17)],
    error=[by_id[i]['hardware_error'] for i in range(1,17)],
    watchdog=[by_id[i]['bus_watchdog_raw'] for i in range(1,17)],
    voltage=[by_id[i]['voltage_raw'] for i in range(1,17)],temperature=[by_id[i]['temperature_c'] for i in range(1,17)])
request=OUT/'saved_post_book_request.json'
request.write_text(json.dumps(dict(plan_version=GraspedYawPlan.PLAN_VERSION,
    motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),origin='offline_diagnostic_from_saved_post_book_readonly_pair',
    off_state=state),indent=2),encoding='utf-8')
rows=[];admission_error=None
try:
    plan=GraspedYawPlan(request);rows=plan.plan(raw)
except (ValueError,RuntimeError) as exc:admission_error=str(exc)
opened,closed=endpoints()
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)

def setup(model):
    d=mujoco.MjData(model);hand={model.body('palm').id}
    for b in range(model.nbody):
        if int(model.body_parentid[b]) in hand:hand.add(b)
    support={model.geom(name).id for name in ('support_glass','support_wood','support_book')}
    def put(q):
        d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(model,d)
        pairs={};self_pairs={}
        for c in d.contact:
            if c.dist>=0:continue
            a,b=int(c.geom1),int(c.geom2);ha=int(model.geom_bodyid[a]) in hand;hb=int(model.geom_bodyid[b]) in hand
            key='|'.join(sorted((model.geom(a).name,model.geom(b).name)))
            if (a in support and hb) or (b in support and ha):pairs[key]=max(pairs.get(key,0.),-float(c.dist)*1000)
            if ha and hb:self_pairs[key]=max(self_pairs.get(key,0.),-float(c.dist)*1000)
        return pairs,self_pairs
    return d,put

records=[];selected=None
for projection in (34.,50.):
    for book_front in (-20.,0.,20.):
        for yhi in (25.,65.):
            model,manifest=build_book(env,OUT/'hypothesis_scene.xml',book_projection_mm=book_front,
                                     book_yhi_mm=yhi,support_projection_mm=projection)
            data,put=setup(model);poses=[]
            for name,q in (('fresh_paired_off',raw),('historical_open',opened),('historical_grasp',closed)):
                support,self_pairs=put(q)
                poses.append(dict(name=name,max_support_model_overlap_mm=max(support.values(),default=0.),
                    support_pairs_mm=support,self_pairs_mm=self_pairs))
            phases={}
            for phase,q in rows:
                support,self_pairs=put(q)
                row=phases.setdefault(phase,dict(max_support_model_overlap_mm=0.,max_self_model_overlap_mm=0.))
                row['max_support_model_overlap_mm']=max(row['max_support_model_overlap_mm'],max(support.values(),default=0.))
                row['max_self_model_overlap_mm']=max(row['max_self_model_overlap_mm'],max(self_pairs.values(),default=0.))
            record=dict(support_projection_mm_hypothesis=projection,book_front_mm_hypothesis=book_front,
                book_yhi_mm_hypothesis=yhi,manifest=manifest,poses=poses,nominal_path_phase_diagnostics=phases)
            records.append(record)
            if projection==50 and book_front==0 and yhi==65:selected=(model,data)
        print(f'20mm book / support {projection:g} / book front {book_front:g}: complete.',flush=True)

# Full, unhulled fixed-palm CAD surface is retained for independent geometric
# landmark inspection. No photo fit or joint-zero correction is installed.
m,d=selected;d.qpos[:16]=ticks_to_sim(raw);mujoco.mj_forward(m,d)
palm=m.body('palm').id;rotation=d.xmat[palm].reshape(3,3);origin=d.xpos[palm]
geom=m.geom('palm_visual').id;mid=int(m.geom_dataid[geom]);va=int(m.mesh_vertadr[mid]);vn=int(m.mesh_vertnum[mid])
vertices=m.mesh_vert[va:va+vn]@d.geom_xmat[geom].reshape(3,3).T+d.geom_xpos[geom]
vertices=(vertices-origin)@rotation*1000
fa=int(m.mesh_faceadr[mid]);fn=int(m.mesh_facenum[mid]);faces=m.mesh_face[fa:fa+fn]
tri=vertices[faces];normal=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);area=np.linalg.norm(normal,axis=1)/2
groups={}
for i in np.flatnonzero((area>1e-9)&(abs(normal[:,1])>np.linalg.norm(normal,axis=1)*.9999)):
    plane=round(float(tri[i,:,1].mean()),2)
    group=groups.setdefault(plane,dict(plane_y_mm=plane,area_mm2=0.,points=[]))
    group['area_mm2']+=float(area[i]);group['points'].extend(tri[i].tolist())
planes=[]
for g in sorted(groups.values(),key=lambda v:v['area_mm2'],reverse=True)[:12]:
    points=np.unique(np.round(g.pop('points'),4),axis=0)
    g.update(bounds_palm_mm=[points.min(0).tolist(),points.max(0).tolist()],vertices_palm_mm=points.tolist());planes.append(g)
(OUT/'fixed_palm_side_planes.json').write_text(json.dumps(planes),encoding='utf-8')
report=dict(status='post_book20_saved_pose_review_complete',source=str(FOLDER.name),
    source_sha256=hashlib.sha256(blob).hexdigest(),book_measurement_sha256=hashlib.sha256(MEASUREMENTS.read_bytes()).hexdigest(),
    raw_ticks=raw.tolist(),max_pair_drift_ticks=source['pair_check']['maximum_drift_ticks'],
    production_plan_admission_passed=admission_error is None,admission_error=admission_error,
    nominal_path_goals=len(rows),book_thickness_mm_user_reported=20.,records=records,
    book_footprint_measured=False,hardware_execution_allowed=False,
    physical_geometry_calibration_verified=False,native_ranges_unchanged=True,hardware_commands_sent=0,
    limitations=['This is 12 footprint hypotheses and a nominal path inspection, not 67-neighborhood/dynamic approval.',
        'Book front and lateral extents are hypotheses and must not be relabeled as measured values.',
        'Source motor readbacks are fresh and OFF, but visual/native geometry calibration remains unverified.'])
(OUT/'result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(dict(status=report['status'],admission_error=admission_error,nominal_path_goals=len(rows),
    cases=[dict(projection=r['support_projection_mm_hypothesis'],book_front=r['book_front_mm_hypothesis'],
        book_yhi=r['book_yhi_mm_hypothesis'],start_overlap_mm=r['poses'][0]['max_support_model_overlap_mm'],
        path_max_support_mm=max((p['max_support_model_overlap_mm'] for p in r['nominal_path_phase_diagnostics'].values()),default=None)) for r in records],
    output=str(OUT/'result.json'),hardware_execution_allowed=False),indent=2),flush=True)
