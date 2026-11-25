"""Exact triangular CAD-vs-CAD query, separate from MuJoCo convex hulls."""
import json,os,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'.analysis_packages'))
import fcl,numpy as np
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from hand_motor_mapping import ticks_to_sim
import horizontal_quarter_plan_20261027 as p

def main():
    plan=p.HorizontalQuarterPlan(p.REQUEST);raw=plan.source_position();out=p.OUT/'exact_mesh_diagnostic';out.mkdir(exist_ok=True)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env,out/'scene.xml',book_projection_mm=-10);correct_model(m,raw)
    names=('cadtip_mf_rf_mf_ds_0','cadtip_mf_rf_rf_ds_0');ids=[m.geom(n).id for n in names];objects=[];meshes=[]
    for g in ids:
        mid=int(m.geom_dataid[g]);va=int(m.mesh_vertadr[mid]);vn=int(m.mesh_vertnum[mid]);fa=int(m.mesh_faceadr[mid]);fn=int(m.mesh_facenum[mid]);vertices=np.array(m.mesh_vert[va:va+vn],dtype=np.float64);faces=np.array(m.mesh_face[fa:fa+fn],dtype=np.int32)
        if faces.min()<0 or faces.max()>=len(vertices):raise RuntimeError('Mesh faces do not index local vertices')
        geometry=fcl.BVHModel();geometry.beginModel(len(vertices),len(faces));geometry.addSubModel(vertices,faces);geometry.endModel();objects.append(fcl.CollisionObject(geometry));meshes.append(dict(geom=m.geom(g).name,mesh=m.mesh(mid).name,vertices=vn,triangles=fn))
        np.savez_compressed(out/(m.geom(g).name+'.npz'),vertices=vertices,faces=faces)
    d=mujoco.MjData(m);results=[]
    rows=[('initial',raw)]+[(phase,q) for phase,q in plan.plan(raw) if phase in p.PREFIX_PHASES]
    for i,(phase,q) in enumerate(rows):
        if i and i%8 and phase==rows[i-1][0]:continue
        d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:]=[0,0,2,1,0,0,0];d.qvel[:]=0;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
        for obj,g in zip(objects,ids):obj.setTransform(fcl.Transform(d.geom_xmat[g].reshape(3,3),d.geom_xpos[g]))
        cr=fcl.CollisionResult();count=fcl.collide(objects[0],objects[1],fcl.CollisionRequest(num_max_contacts=20,enable_contact=True),cr)
        dr=fcl.DistanceResult();distance=fcl.distance(objects[0],objects[1],fcl.DistanceRequest(enable_nearest_points=True),dr)
        row=dict(index=i,phase=phase,triangle_collision_count=int(count),distance_mm=float(distance*1000),nearest_points_mm=[np.asarray(v).tolist() for v in dr.nearest_points],geom_positions=[d.geom_xpos[g].tolist() for g in ids]);results.append(row)
        if i==0:print('INITIAL',json.dumps(row),flush=True)
    report=dict(status='exact_triangular_mesh_diagnostic',library='python-fcl0.7.0.11',meshes=meshes,observations=results,sampled_poses=len(results),any_triangle_collision=any(v['triangle_collision_count'] for v in results),minimum_distance_mm=min(v['distance_mm'] for v in results),hardware_execution_allowed=False,devices_opened=False)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k!='observations'}),flush=True)
if __name__=='__main__':main()
