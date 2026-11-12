"""Exploratory path adaptation, NOT calibration and NOT motor execution.

Original recorded poses are immutable teacher references. Only a new offline
trajectory is optimized inside native ranges; no joint zero/mesh/tolerance is
changed. Middle/ring mesh hulls supplement existing native collision checks.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse
import json
from pathlib import Path
from types import SimpleNamespace
import copy
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from scipy.optimize import minimize
from mujoco_playground import registry
from book_support_model_20261024 import build_book
from hand_motor_mapping import hw_to_sim, sim_to_ticks
from review_original_sequence_20261024 import load_source, ROOT

OUT=ROOT/'right_hand_rotation_20261024/original_sequence_clearance'


def build():
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    assets=copy.deepcopy(dict(env._model_assets))
    for name,blob in list(assets.items()):
        if not str(name).endswith('.xml'):continue
        xml=ET.fromstring(blob)
        for body in xml.iter('body'):
            bn=body.get('name','');mf=bn.startswith('mf_');rf=bn.startswith('rf_')
            if not (mf or rf):continue
            for i,geom in enumerate(list(body.findall('geom'))):
                if geom.get('mesh') is None:continue
                g=copy.deepcopy(geom);g.attrib.pop('class',None)
                g.attrib.update(name=f'adapt_mf_rf_{bn}_{i}',type='mesh',density='0',mass='0',group='4',rgba='0 0 0 0',
                    contype='64' if mf else '128',conaffinity='128' if mf else '64')
                body.append(g)
        assets[name]=ET.tostring(xml,encoding='utf-8')
    wrapped=SimpleNamespace(mj_model=env.mj_model,xml_path=env.xml_path,_model_assets=assets)
    model,manifest=build_book(wrapped,OUT/'scene.xml',book_projection_mm=-10.,book_ylo_mm=-500.,book_yhi_mm=500.,support_projection_mm=50.)
    proxies=[g for g in range(model.ngeom) if model.geom(g).name.startswith('adapt_mf_rf_')]
    if len(proxies)!=10 or any(model.geom_type[g]!=mujoco.mjtGeom.mjGEOM_MESH for g in proxies):
        raise RuntimeError('Middle/ring proxy must preserve the visual mesh geometry')
    manifest['middle_ring_mesh_model_revision']='explicit_mesh_type_v1_20261025'
    return model,manifest


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--cad-tips',action='store_true');args=parser.parse_args()
    OUT.mkdir(parents=True,exist_ok=True)
    names,hw,source=load_source();teacher=hw_to_sim(hw)
    if args.cad_tips:
        from fingertip_cad_model_20261025 import build as build_cad
        cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
        m,manifest=build_cad(env,OUT/'CAD_adaptation_scene.xml')
    else:m,manifest=build()
    d=mujoco.MjData(m);lo,hi=m.jnt_range[:16].T.copy()
    cube=m.geom('cube').id;m.geom_contype[cube]=m.geom_conaffinity[cube]=0
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    supports={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    def place(q):
        d.qpos[:16]=q;d.qpos[-7:]=[10,10,10,1,0,0,0];d.qvel[:]=0;mujoco.mj_forward(m,d)
    def contacts(q):
        place(q);pairs={}
        for c in d.contact:
            if c.dist>=.0005:continue
            a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if ha and hb or a in supports and hb or b in supports and ha:
                pair=tuple(sorted((a,b)));pairs[pair]=min(pairs.get(pair,1.),float(c.dist))
        return pairs
    def worst(q):return max([-v for v in contacts(q).values()]+[0.])
    history=[]
    def adapt(ref,start,previous=None):
        pairs=set(contacts(ref))|set(contacts(start));x=start.copy();attempts=[]
        for outer in range(6):
            ordered=sorted(pairs)
            if not ordered:
                # An obstacle-free reference is the destination, not a request
                # to keep the previous warm start unchanged.
                x=ref.copy()
                break
            def constraints(q):
                place(q)
                return np.array([mujoco.mj_geomDistance(m,d,a,b,.05,None)-.0005 for a,b in ordered])*1000
            def objective(q):
                value=np.sum(((q-ref)/.2)**2)
                if previous is not None:value+=.15*np.sum(((q-previous)/.2)**2)
                return value
            def objective_jac(q):
                gradient=2*(q-ref)/.04
                if previous is not None:gradient+=.3*(q-previous)/.04
                return gradient
            def constraints_jac(q):
                place(q);rows=[]
                for a,b in ordered:
                    witness=np.zeros(6);distance=mujoco.mj_geomDistance(m,d,a,b,.05,witness)
                    if abs(distance)>=.049 or abs(distance)<1e-10:
                        rows.append(np.zeros(16));continue
                    normal=(witness[3:]-witness[:3])/distance
                    ja,jb=np.zeros((3,m.nv)),np.zeros((3,m.nv))
                    mujoco.mj_jac(m,d,ja,None,witness[:3],int(m.geom_bodyid[a]))
                    mujoco.mj_jac(m,d,jb,None,witness[3:],int(m.geom_bodyid[b]))
                    rows.append((normal@(jb-ja))[:16]*1000)
                return np.array(rows)
            bounds=list(zip(np.maximum(lo,ref-.4),np.minimum(hi,ref+.4)))
            result=minimize(objective,np.clip(x,np.array(bounds)[:,0],np.array(bounds)[:,1]),
                method='SLSQP',jac=objective_jac,bounds=bounds,
                constraints=[dict(type='ineq',fun=constraints,jac=constraints_jac)] if args.cad_tips else [dict(type='ineq',fun=constraints)],
                options=dict(maxiter=150,ftol=1e-7,eps=1e-4))
            x=result.x
            found=contacts(x);pairs.update(found)
            attempts.append(dict(iteration=outer,optimizer_success=bool(result.success),message=str(result.message),
                max_overlap_mm=worst(x)*1000,active_pair_count=len(pairs)))
            if worst(x)<=1e-7:break
        return x,dict(attempts=attempts,maximum_teacher_deviation_deg=float(np.degrees(np.max(abs(x-ref)))),
            residual_overlap_mm=worst(x)*1000,
            pairs=[dict(geoms=[m.geom(a).name,m.geom(b).name],distance_mm=v*1000) for (a,b),v in contacts(x).items()])
    poses=[]
    for i,ref in enumerate(teacher):
        adapted,review=adapt(ref,ref)
        poses.append(adapted)
        history.append(dict(stage=i,name=names[i],review=review,raw_goal_ticks=sim_to_ticks(adapted).tolist()))
        print('ADAPTED_POSE',i,'teacher_delta_deg',round(review['maximum_teacher_deviation_deg'],2),
            'remaining_model_overlap_mm',round(review['residual_overlap_mm'],4),flush=True)
    path=[poses[0]];parameters=[0.];failures=[]
    for stage in range(1,3 if args.cad_tips else 7):
        for k in range(1,33):
            fraction=k/32;ref=teacher[stage-1]+fraction*(teacher[stage]-teacher[stage-1])
            guess=poses[stage-1]+fraction*(poses[stage]-poses[stage-1])
            adapted,review=adapt(ref,path[-1] if args.cad_tips else guess,path[-1]);path.append(adapted)
            parameters.append(stage-1+fraction)
            if review['residual_overlap_mm']>.001:failures.append(dict(stage=stage,fraction=fraction,review=review))
        print('ADAPTED_SEGMENT',stage,'failures',len(failures),flush=True)
    # Verify interpolation rather than only solved keyframes, at <=2 tick steps.
    if args.cad_tips:
        # Refine colliding chords, without accepting a colliding interpolation
        # between individually feasible configurations.
        for refinement in range(6):
            refined=[path[0]];refined_parameters=[parameters[0]];inserted=0
            for a,b,ta,tb in zip(path[:-1],path[1:],parameters[:-1],parameters[1:]):
                ra,rb=sim_to_ticks(a),sim_to_ticks(b)
                count=max(1,int(np.ceil(np.max(abs(rb-ra))/2)))
                segment=[np.rint(ra+(rb-ra)*k/count).astype(int) for k in range(1,count)]
                from hand_motor_mapping import ticks_to_sim
                blocked=any(worst(q)*1000>.001 for q in ticks_to_sim(np.array(segment).reshape(-1,16))) if segment else False
                if blocked:
                    t=(ta+tb)/2;stage=min(int(np.floor(t)),1);f=t-stage
                    ref=teacher[stage]+f*(teacher[stage+1]-teacher[stage])
                    q,_=adapt(ref,a,previous=a)
                    refined.append(q);refined_parameters.append(t);inserted+=1
                refined.append(b);refined_parameters.append(tb)
            path,parameters=refined,refined_parameters
            print('CAD_CHORD_REFINEMENT',refinement,'inserted',inserted,flush=True)
            if not inserted:break
    rounded=sim_to_ticks(np.array(path));dense=[rounded[0]]
    for a,b in zip(rounded[:-1],rounded[1:]):
        steps=max(1,int(np.ceil(np.max(abs(b-a))/2)))
        dense.extend(np.rint(a+(b-a)*k/steps).astype(int) for k in range(1,steps+1))
    from hand_motor_mapping import ticks_to_sim
    max_depth=0.;bad=[]
    for i,q in enumerate(ticks_to_sim(np.array(dense))):
        depth=worst(q)*1000;max_depth=max(max_depth,depth)
        if depth>.001:bad.append(dict(index=i,model_overlap_mm=depth))
    result=dict(source=source,teacher_names=names,teacher_source_changed=False,
        original_goal_poses_sim_rad=teacher.tolist(),adapted_endpoints_sim_rad=np.array(poses).tolist(),
        endpoint_reviews=history,sparse_path_goal_ticks=rounded.tolist(),sparse_path_teacher_parameters=parameters,
        dense_goal_ticks=np.array(dense).tolist(),
        sparse_failures=failures,dense_path_max_model_overlap_mm=max_depth,
        dense_path_overlap_samples=bad,native_joint_ranges_unchanged=True,joint_zero_offsets_fitted=False,
        contact_tolerances_unchanged=True,maximum_teacher_deviation_bound_rad=.4,
        full_static_path_clearance_satisfied=not bad and not failures,
        cube_dynamics_tested=False,hardware_execution_allowed=False,hardware_commands_sent=0,
        physical_ninety_degree_rotation_verified=False,support=manifest,
        revision=2,subdivisions_per_stage=32,optimizer_finite_difference_rad=1e-4)
    result['uses_CAD_fingertip_contacts']=bool(args.cad_tips)
    result['analytic_distance_jacobians']=bool(args.cad_tips)
    if args.cad_tips:
        result['last_target_reached']=bool(np.array_equal(rounded[-1],sim_to_ticks(teacher[2])))
        if not result['last_target_reached']:raise RuntimeError('Teacher endpoint was not reached')
        deviations=[]
        for q,t in zip(path,parameters):
            stage=min(int(np.floor(t)),1);f=t-stage
            ref=teacher[stage]+f*(teacher[stage+1]-teacher[stage])
            deviations.append(float(np.max(abs(q-ref))))
        result['actual_maximum_teacher_deviation_rad']=max(deviations)
        if max(deviations)>.400001:raise RuntimeError('Adapted trajectory exceeded its reference deviation bound')
    output='CAD_adapted_forward_reference_path_v2.json' if args.cad_tips else 'adapted_reference_path_v2.json'
    (OUT/output).write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('SUMMARY',json.dumps(dict(goals=len(dense),dense_max_overlap_mm=max_depth,
        static_path_valid=result['full_static_path_clearance_satisfied'],cube_dynamics_tested=False)),flush=True)


if __name__=='__main__':main()
