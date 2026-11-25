"""Endpoint-only opposing fingertip feasibility, not a motion certificate."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,itertools,json,time
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from loaded_quarter_reentry_20261025 import limits
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261026/yaw_after_flip_v1'
def main():
    global OUT
    parser=argparse.ArgumentParser();parser.add_argument('--corners',action='store_true');parser.add_argument('--middle-front',action='store_true');parser.add_argument('--all-sides',action='store_true');parser.add_argument('--seed-folder');args=parser.parse_args()
    if args.seed_folder:OUT=(ROOT/args.seed_folder).resolve();OUT.relative_to(ROOT)
    seed=json.loads((OUT/'seed.json').read_bytes());base=np.asarray(seed['commanded_hand_ticks']);residual=np.asarray(seed['response_residual_ticks']);cube_pose=np.asarray(seed['settled_cube_qpos'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'pinch_scene.xml');set_cube_physics(m,.055,.1);d=mujoco.MjData(m)
    cube=m.geom('cube').id;C=cube_pose[:3];B=Rotation.from_quat(cube_pose[-4:][[1,2,3,0]]).as_matrix();up_local=np.asarray(seed.get('up_cube_normal',[1,0,0]));up=B@up_local;side_axes=[a for a in range(3) if abs(up_local[a])<.5]
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')};lo,hi=limits()
    def put(goal):
        d.qpos[:16]=ticks_to_sim(goal+residual);d.qpos[-7:]=cube_pose;d.qvel[:]=0;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
    def defects(allowed_tip=None):
        sp=sup=wrong_cube=0.
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);dep=max(0.,-float(c.dist)*1000);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if ha and hb:sp=max(sp,dep)
            if (a in support and hb) or (b in support and ha):sup=max(sup,dep)
            if cube in (a,b) and allowed_tip not in (a,b):wrong_cube=max(wrong_cube,dep)
        return sp,sup,wrong_cube
    results=[];best={}
    for finger,ids in (('if',np.array([7,5,6,4])),('mf',np.array([11,9,10,8]))):
        geom=m.geom(finger+'_tip').id
        entries=([(1 if finger=='if' or args.middle_front else 2,1,z,offset) for z,offset in itertools.product((.005,.015,.022),(-.015,0.,.015))]
            if args.corners else [(axis,sign,z,0.) for axis,sign,z in itertools.product((1,2),(-1,1),(-.005,.005))])
        if args.all_sides:entries=list(itertools.product(side_axes,(-1,1),(0.,.01),(-.008,.008)))
        for axis,sign,z,offset in entries:
            normal=B[:,axis]*sign
            tangent=np.cross(normal,up);tangent/=np.linalg.norm(tangent)
            target=C+normal*.0365+up*z+tangent*offset
            solutions=[]
            for root,pip,dip in ((2500,2600,2600),(2700,2900,2700),(3000,2900,2500),(2300,2700,2900)):
                initial=base[ids].astype(float);initial[[0,2,3]]=[root,pip,dip];initial=np.clip(initial,lo[ids]+.01,hi[ids]-.01)
                def objective(value):
                    q=base.astype(float).copy();q[ids]=value;put(q)
                    center=d.geom_xpos[geom];gap=mujoco.mj_geomDistance(m,d,cube,geom,1.,None)*1000
                    sp,sup,wrong=defects(geom)
                    # Position identifies the intended side; the exact surface distance
                    # may differ from the CAD mesh centroid by several millimetres.
                    tangential=np.array([(center-target)@tangent,(center-target)@up])*1000
                    wrong_side=max(0.,.0275-(center-C)@normal)*1000
                    return np.r_[tangential,10*(gap-.1),10*sp,10*sup,10*wrong,5*wrong_side,.001*(value-base[ids])]
                fit=least_squares(objective,initial,bounds=(lo[ids]+.001,hi[ids]-.001),diff_step=.00004,max_nfev=160)
                q=base.copy();q[ids]=np.rint(fit.x).astype(int);put(q);gap=float(mujoco.mj_geomDistance(m,d,cube,geom,1.,None)*1000)
                sp,sup,wrong=defects(geom);center=d.geom_xpos[geom];position_error=float(1000*np.linalg.norm([(center-target)@tangent,(center-target)@up]))
                # Integer goals may make a rounded solution slightly intersect.
                # Search neighbours instead of relaxing any contact criterion.
                if args.corners and position_error<=8 and -.5<=gap<=4 and (sp>.01 or wrong>1):
                    rounded=q.copy();repairs=[]
                    for delta in itertools.product((-1,0,1),repeat=4):
                        candidate=rounded.copy();candidate[ids]+=delta
                        if np.any(candidate<lo) or np.any(candidate>hi):continue
                        put(candidate);ngap=float(mujoco.mj_geomDistance(m,d,cube,geom,1.,None)*1000);ns,nu,nw=defects(geom)
                        center=d.geom_xpos[geom];ne=float(1000*np.linalg.norm([(center-target)@tangent,(center-target)@up]))
                        if -.3<=ngap<=3 and ne<=6 and ns<=.01 and nu<=.15 and nw<=1:repairs.append((ne+abs(ngap),candidate.copy(),ngap,ns,nu,nw,ne))
                    if repairs:_,q,gap,sp,sup,wrong,position_error=min(repairs,key=lambda v:v[0])
                    put(q);center=d.geom_xpos[geom]
                row=dict(finger=finger,side_axis=axis,side_sign=sign,height_mm=z*1000,tangent_offset_mm=offset*1000,goal_ticks=q.tolist(),
                    gap_mm=gap,position_error_mm=position_error,self_overlap_mm=sp,support_overlap_mm=sup,other_cube_overlap_mm=wrong,
                    tip_center_world=center.tolist())
                row['endpoint_promising']=bool(-.3<=gap<=3 and position_error<=6 and sp<=.01 and sup<=.15 and wrong<=1)
                solutions.append(row)
            chosen=min(solutions,key=lambda v:v['position_error_mm']+abs(v['gap_mm'])+10*v['self_overlap_mm']+10*v['support_overlap_mm']+5*v['other_cube_overlap_mm'])
            results.append(chosen);best[(finger,axis,sign,z)]=chosen
            print('ENDPOINT',json.dumps({k:v for k,v in chosen.items() if k not in ('goal_ticks','tip_center_world')}),flush=True)
    pairs=[]
    for axis,z,sign in ([] if args.corners else itertools.product((1,2),(-.005,.005),(-1,1))):
        a=best[('if',axis,sign,z)];b=best[('mf',axis,-sign,z)];q=base.copy();q[[7,5,6,4]]=np.asarray(a['goal_ticks'])[[7,5,6,4]];q[[11,9,10,8]]=np.asarray(b['goal_ticks'])[[11,9,10,8]];put(q)
        gaps={finger:float(mujoco.mj_geomDistance(m,d,cube,m.geom(finger+'_tip').id,1.,None)*1000) for finger in ('if','mf')}
        sp,sup,wrong=defects();row=dict(axis=axis,height_mm=z*1000,index_side=sign,goal_ticks=q.tolist(),gaps_mm=gaps,self_overlap_mm=sp,support_overlap_mm=sup,cube_overlap_mm=wrong,
            endpoint_promising=bool(a['endpoint_promising'] and b['endpoint_promising'] and sp<=.01 and sup<=.15 and wrong<=1))
        pairs.append(row)
    adjacent=[]
    for a,b in itertools.product([v for v in results if v['finger']=='if' and v['endpoint_promising']],[v for v in results if v['finger']=='mf' and v['endpoint_promising']]):
        if a['side_axis']==b['side_axis'] and a['side_sign']==b['side_sign']:continue
        q=base.copy();q[[7,5,6,4]]=np.asarray(a['goal_ticks'])[[7,5,6,4]];q[[11,9,10,8]]=np.asarray(b['goal_ticks'])[[11,9,10,8]];put(q)
        sp,sup,wrong=defects();gaps={finger:float(mujoco.mj_geomDistance(m,d,cube,m.geom(finger+'_tip').id,1.,None)*1000) for finger in ('if','mf')}
        adjacent.append(dict(index_side=[a['side_axis'],a['side_sign']],middle_side=[b['side_axis'],b['side_sign']],goal_ticks=q.tolist(),
            self_overlap_mm=sp,support_overlap_mm=sup,cube_overlap_mm=wrong,gaps_mm=gaps,endpoint_promising=bool(sp<=.01 and sup<=.15 and wrong<=1 and all(-.3<=v<=3 for v in gaps.values()))))
    report=dict(status='contact_gap_refined_fingertip_endpoint_feasibility_only',candidates=results,combined=pairs,adjacent_pairs=adjacent,
        position_metric='two tangential coordinates; radial distance determined by actual CAD-to-cube surface gap',
        source_seed='seed.json',model=manifest,initial_cube_pose_assumed=True,transition_path_checked=False,
        free_cube_grasp_verified=False,hardware_execution_allowed=False,real_devices_opened=False)
    (OUT/('pinch_upper_corner_endpoints.json' if args.corners else 'pinch_contact_gap_refined.json')).write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print('SUMMARY',json.dumps(dict(single_promising=sum(v['endpoint_promising'] for v in results),paired_promising=sum(v['endpoint_promising'] for v in pairs),adjacent_promising=sum(v['endpoint_promising'] for v in adjacent))),flush=True)
if __name__=='__main__':main()
