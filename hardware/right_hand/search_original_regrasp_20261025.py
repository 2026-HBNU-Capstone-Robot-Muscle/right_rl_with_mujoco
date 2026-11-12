"""Offline regrasp configurations after the recorded horizontal push.

Cube pose is fixed ONLY for inverse-kinematics target construction. It is not
a physics rollout or evidence of rotation. Joint zeros, meshes and native joint
limits are unchanged. Every reported candidate is separately collision checked.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
import numpy as np
import mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from adapt_original_sequence_clearance_20261024 import build, OUT
from hand_motor_mapping import ticks_to_sim, sim_to_ticks
from sac_cube_model import set_cube_physics


def main():
    prior=json.loads((OUT/'whole_ninety_finish_search.json').read_bytes())
    baseline=prior['held_baseline_control']
    prefix=json.loads((OUT/'horizontal_reference_primitive.json').read_bytes())
    q0=ticks_to_sim(prefix['goal_ticks'][-1])
    original_reference_q=q0.copy()
    # A new trajectory waypoint, not a change to motor zero or joint range.
    # Clearing the ring before regrasp removes a swept middle/ring crossing.
    q0[8:12]=[.2,0.,0.,.4]
    m,_=build();set_cube_physics(m,side_m=.055,mass_kg=.1);d=mujoco.MjData(m)
    cube=m.geom('cube').id;center=np.array(baseline['cube_center_m']);quat=np.array(baseline['cube_quat_wxyz'])
    R=Rotation.from_quat(quat[[1,2,3,0]]).as_matrix()
    moving=np.array([0,1,2,3,4,5,6,7,12,13,14,15]);lo,hi=m.jnt_range[:16].T
    tips=[m.geom(n).id for n in ('if_tip','mf_tip','th_tip')]
    # Index and middle oppose across local Z; thumb approaches the local -X face.
    normals_local=np.array([[0,0,-1.],[0,0,1.],[-1.,0,0]])
    normals=normals_local@R.T;targets=center+normals*.0275
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    cube_geoms=[g for g in range(m.ngeom) if int(m.geom_bodyid[g]) in hand and
        ((int(m.geom_contype[g])&int(m.geom_conaffinity[cube])) or
         (int(m.geom_contype[cube])&int(m.geom_conaffinity[g])))]
    def place(q,rotation=R):
        d.qpos[:16]=q;d.qpos[-7:]=np.r_[center,Rotation.from_matrix(rotation).as_quat()[[3,0,1,2]]]
        d.qvel[:]=0;mujoco.mj_forward(m,d)
    def collisions(q):
        place(q);pairs={};cube_overlap=0.
        for c in d.contact:
            if c.dist>=-1e-7:continue
            a,b=int(c.geom1),int(c.geom2)
            ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if ha and hb or a in support and hb or b in support and ha:
                pair=tuple(sorted((a,b)));pairs[pair]=min(pairs.get(pair,0.),float(c.dist))
            if cube in (a,b):cube_overlap=max(cube_overlap,-float(c.dist))
        return pairs,cube_overlap
    def support_point(g,normal):
        rotation=d.geom_xmat[g].reshape(3,3);w=-np.asarray(normal);local=rotation.T@w
        size=m.geom_size[g];kind=int(m.geom_type[g])
        if kind==int(mujoco.mjtGeom.mjGEOM_SPHERE):delta=w*size[0]
        elif kind==int(mujoco.mjtGeom.mjGEOM_CAPSULE):delta=w*size[0]+rotation[:,2]*size[1]*np.sign(local[2])
        elif kind==int(mujoco.mjtGeom.mjGEOM_ELLIPSOID):
            delta=rotation@(size*size*local/max(np.linalg.norm(size*local),1e-12))
        elif kind==int(mujoco.mjtGeom.mjGEOM_BOX):delta=rotation@(size*np.sign(local))
        else:raise ValueError('Unsupported fingertip collision primitive: '+str(kind))
        return d.geom_xpos[g]+delta
    def surface_gap(g,normal,target):
        rotation=d.geom_xmat[g].reshape(3,3);size=m.geom_size[g];local=rotation.T@normal
        kind=int(m.geom_type[g])
        if kind==int(mujoco.mjtGeom.mjGEOM_BOX):radius=np.sum(abs(local)*size)
        elif kind==int(mujoco.mjtGeom.mjGEOM_ELLIPSOID):radius=np.linalg.norm(local*size)
        elif kind==int(mujoco.mjtGeom.mjGEOM_SPHERE):radius=size[0]
        elif kind==int(mujoco.mjtGeom.mjGEOM_CAPSULE):radius=size[0]+size[1]*abs(local[2])
        else:raise ValueError('Unsupported collision primitive')
        offset=d.geom_xpos[g]-target
        gap=offset@normal-radius
        tangential=offset-normal*(offset@normal)
        return gap,tangential
    print('TIP_GEOMETRY',[(m.geom(g).name,int(m.geom_type[g]),m.geom_size[g].tolist()) for g in tips],flush=True)
    rng=np.random.default_rng(20261025);active=set();seeds=[q0.copy()]
    # The previously reached grip is a seed, not an assumed contact solution.
    grip=np.array([2986,2942,2913,2151,3237,2063,2233,2473,3059,2452,2456,2594,3238,2144,2625,3053])
    seed=q0.copy();seed[moving]=ticks_to_sim(grip)[moving];seeds.append(seed)
    for _ in range(14):
        seed=q0.copy()
        seed[:8]=[rng.uniform(.3,1.2),rng.uniform(-.5,.5),rng.uniform(.3,1.6),rng.uniform(.4,1.9),
                  rng.uniform(.3,1.2),rng.uniform(-.7,.3),rng.uniform(.3,1.6),rng.uniform(.4,1.9)]
        seed[12:]=rng.uniform(lo[12:],hi[12:]);seeds.append(np.clip(seed,lo,hi))
    for q in seeds:
        active.update(collisions(q)[0])
    results=[]
    for index,seed in enumerate(seeds):
        x=seed[moving].copy();attempts=[]
        for outer in range(3):
            pairs=sorted(active)
            def residual(values):
                q=q0.copy();q[moving]=values;place(q)
                terms=[]
                for g,n,p in zip(tips,normals,targets):
                    gap,tangential=surface_gap(g,n,p)
                    terms.extend([gap/.002,*list(tangential/.020)])
                tip=np.array(terms)
                penalty=np.array([max(0.,.0005-mujoco.mj_geomDistance(m,d,a,b,.1,None))/.001
                    for a,b in pairs])
                cube_penalty=np.array([max(0.,-mujoco.mj_geomDistance(m,d,g,cube,.1,None))/.001
                    for g in cube_geoms])
                # These are trajectory targets, not fitted encoder zero offsets.
                return np.r_[tip,penalty*3.,cube_penalty*5.,.01*(values-seed[moving])]
            result=least_squares(residual,x,bounds=(lo[moving]+1e-6,hi[moving]-1e-6),
                diff_step=1e-4,max_nfev=140,ftol=1e-6,xtol=1e-6,gtol=1e-6)
            x=result.x;q=q0.copy();q[moving]=x;found,overlap=collisions(q);active.update(found)
            place(q);errors=[abs(mujoco.mj_geomDistance(m,d,g,cube,.1,None))*1000 for g in tips]
            max_self=max([-v for v in found.values()]+[0.])*1000
            attempts.append(dict(maximum_target_surface_error_mm=max(errors),self_overlap_mm=max_self,cube_overlap_mm=overlap*1000))
            if max_self<.001:break
        row=dict(seed=index,goal_ticks=sim_to_ticks(q).tolist(),tip_surface_errors_mm=errors,
            maximum_tip_surface_error_mm=max(errors),self_overlap_mm=max_self,cube_overlap_mm=overlap*1000,
            attempts=attempts,geometric_grasp_candidate=bool(max(errors)<3 and max_self<.001 and overlap<.001))
        results.append(row)
        print('REGRASP',index,'tip_mm',[round(v,2) for v in errors],'self_mm',round(max_self,3),
            'cube_mm',round(overlap*1000,3),'candidate',row['geometric_grasp_candidate'],flush=True)
    results.sort(key=lambda r:(not r['geometric_grasp_candidate'],r['maximum_tip_surface_error_mm']+r['self_overlap_mm']*5))
    out=dict(source='whole_ninety_finish_search.json',cube_center_m=center.tolist(),cube_quat_wxyz=quat.tolist(),
        target_face_normals_world=normals.tolist(),target_surface_points_world_m=targets.tolist(),
        candidates=results,native_joint_limits_unchanged=True,joint_zero_offsets_fitted=False,
        cube_fixed_only_for_IK_not_simulated_success=True,free_cube_dynamic_replay_pending=True,
        hardware_execution_allowed=False,hardware_commands_sent=0,physical_ninety_degree_rotation_verified=False)
    out.update(revision=3,whole_hand_cube_interpenetration_penalty_included=True,
        fingertip_plane_gap_and_tangential_center_objective=True,
        original_reference_sim_rad=original_reference_q.tolist(),
        ring_clearance_target_sim_rad=q0[8:12].tolist(),
        ring_clearance_required_before_regrasp=True)
    (OUT/'regrasp_configuration_search_v3.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
    print('SUMMARY','geometric_candidates',sum(r['geometric_grasp_candidate'] for r in results),flush=True)


if __name__=='__main__':main()
