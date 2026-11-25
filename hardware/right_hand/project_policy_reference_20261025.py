"""Project archived simulation joint targets into the full CAD contact model."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib
from pathlib import Path
import numpy as np
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from collision_projection_20261025 import CollisionProjection
from hand_motor_mapping import sim_to_ticks,ticks_to_sim
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/corrected_policy_reference'

def main():
    blob=(ROOT/'sac_reachable_trace_search_result.json').read_bytes();source=json.loads(blob)
    teacher=np.asarray(next(r for r in source['results'] if r['compensation_rad']==.07 and r['time_scale']==1.)['goals_sim_order'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    model,manifest=build(env,OUT/'projected_policy_CAD.xml');projector=CollisionProjection(model)
    fixture=np.argmax([projector.maximum_overlap(q) for q in teacher]);check=projector.check_jacobian(teacher[fixture])
    print('DISTANCE_JACOBIAN',check,flush=True)
    if not check['passes_smooth_fixture_check']:raise RuntimeError('Analytic distance gradient fixture failed')
    path=[];reviews=[];parameters=[]
    for i,q in enumerate(teacher):
        qq,review=projector.project(q,path[-1] if path else q)
        path.append(qq);parameters.append(float(i));reviews.append(review)
        if i%10==0:print('PROJECT_POLICY',i,'overlap',review['maximum_overlap_mm'],'deviation_deg',np.degrees(review['maximum_reference_deviation_rad']),flush=True)
    for refinement in range(5):
        new=[path[0]];newt=[parameters[0]];inserted=0
        for a,b,ta,tb in zip(path[:-1],path[1:],parameters[:-1],parameters[1:]):
            ra,rb=sim_to_ticks(a),sim_to_ticks(b);count=max(1,int(np.ceil(np.max(abs(rb-ra))/2)))
            samples=ticks_to_sim(np.rint(ra+(rb-ra)*np.arange(1,count)[:,None]/count).astype(int))
            if any(projector.maximum_overlap(s)>1e-7 for s in samples):
                t=(ta+tb)/2;ref=np.array([np.interp(t,np.arange(len(teacher)),teacher[:,j]) for j in range(16)])
                qq,_=projector.project(ref,(a+b)/2);new.append(qq);newt.append(t);inserted+=1
            new.append(b);newt.append(tb)
        path,parameters=new,newt
        print('PROJECT_CHORDS',refinement,'inserted',inserted,flush=True)
        if not inserted:break
    raw=sim_to_ticks(np.array(path));dense=[raw[0]];dense_t=[parameters[0]]
    for a,b,ta,tb in zip(raw[:-1],raw[1:],parameters[:-1],parameters[1:]):
        n=max(1,int(np.ceil(np.max(abs(b-a))/2)))
        for k in range(1,n+1):dense.append(np.rint(a+(b-a)*k/n).astype(int));dense_t.append(ta+(tb-ta)*k/n)
    depth=max(projector.maximum_overlap(s) for s in ticks_to_sim(np.array(dense)))*1000
    result=dict(source_sha256=hashlib.sha256(blob).hexdigest(),original_source_unchanged=True,
        teacher_sim_goals_rad=teacher.tolist(),projected_goals_sim_rad=np.array(path).tolist(),
        sparse_source_parameters=parameters,dense_raw_ticks=np.array(dense).tolist(),dense_source_parameters=dense_t,
        maximum_dense_model_overlap_mm=depth,full_static_path_clear=depth<.0001,
        reviews=reviews,distance_gradient_check=check,model=manifest,
        joint_zero_offsets_changed=False,native_joint_ranges_changed=False,collision_geometry_changed=False,
        collision_tolerances_changed=False,projection_bound_rad=.45,hardware_execution_allowed=False,
        hardware_commands_sent=0,physical_ninety_degree_rotation_verified=False)
    (OUT/'projected_policy_reference.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('PROJECTED_POLICY_COMPLETE','dense_goals',len(dense),'overlap_mm',depth,flush=True)

if __name__=='__main__':main()
