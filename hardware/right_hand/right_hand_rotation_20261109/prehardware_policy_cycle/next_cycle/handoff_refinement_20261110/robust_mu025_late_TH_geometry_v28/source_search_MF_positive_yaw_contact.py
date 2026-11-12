"""MF lower-face contact proposals with a positive yaw moment arm.

Only IK copies fix the cube. Native free-object arrival is a separate test.
"""
from pathlib import Path
import argparse
import json
import numpy as np
import mujoco
from scipy.optimize import differential_evolution
from scipy.spatial import ConvexHull
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,SIM_TO_MOTOR_IDS,TICK,save_json,sha
from upper_layer_cylinder_clearance_20261105 import precompute


def main():
    p=argparse.ArgumentParser();p.add_argument('--tag',default='actual_A_next_MF_positive_yaw_geometry');p.add_argument('--finger',choices=('if','mf','rf','th'),default='mf');p.add_argument('--allow-upper-contact',action='store_true');p.add_argument('--source-state');p.add_argument('--sliding-friction',type=float);args=p.parse_args()
    if not args.tag.replace('_','').isalnum():raise ValueError('Local tag required')
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/args.tag;source=base/'actual_A_next_MF_reset376/pip-30_dip15_mcp10/end_state.npz'
    if args.source_state:
        source=(OUT/args.source_state).resolve()
        if not source.is_relative_to(OUT.resolve()):raise ValueError('Local actual source state required')
    if folder.exists():raise RuntimeError('Preserve prior directed MF-contact search')
    model,initial,_,_,provenance=assets('active_ccd50')
    if args.sliding_friction is not None:
        if not .2<=args.sliding_friction<=.4:raise ValueError('Bounded offline friction hypothesis required')
        model.geom_friction[:,0]=args.sliding_friction
    z=dict(np.load(source,allow_pickle=False));d=restore(model,z);g=Guard(model);cube=model.geom('cube').id;tip=model.geom(args.finger+'_tip').id;C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();up=initial['up'];start=4*('if','mf','rf','th').index(args.finger);active=np.arange(start,start+4);sid=np.asarray(SIM_TO_MOTOR_IDS)-1;lag=d.ctrl-d.qpos[:16]
    low=np.maximum(g.qlo,(np.ceil(g.lo[sid])+2-2048)*TICK-lag);high=np.minimum(g.qhi,(np.floor(g.hi[sid])-2-2048)*TICK-lag)
    vertices=next(v for gid,v,_ in precompute(model,g.hand) if gid==tip);hull=ConvexHull(vertices);A=hull.equations[:,:3];b=-hull.equations[:,3]
    folder.mkdir();(folder/'source_search_MF_positive_yaw_contact.py').write_bytes(Path(__file__).read_bytes());rows=[]
    for index,(axis,sign) in enumerate(((0,1),(0,-1),(2,1),(2,-1))):
        across=2 if axis==0 else 0;across_sign=sign if axis==0 else -sign;normal=sign*B[:,axis];ik=restore(model,z)
        def measure(x):
            ik.qpos[:]=z['qpos'];ik.qpos[active]=x[:4];ik.ctrl[:]=ik.qpos[:16]+lag;mujoco.mj_forward(model,ik)
            local_point=np.zeros(3);local_point[axis]=sign*27.45;local_point[1]=x[4];local_point[across]=across_sign*x[5];point=C+B@local_point/1000
            R=ik.geom_xmat[tip].reshape(3,3);local=R.T@(point-ik.geom_xpos[tip]);planes=A@local-b;j=int(np.argmax(planes));gap=float(planes[j]*1000);err=float(np.linalg.norm(R@A[j]+normal));support,own=g.contacts(ik)
            pen=max((-float(c.dist)*1000 for c in ik.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            return gap,err,support,own,pen,local_point
        def objective(x):
            gap,err,support,own,pen,_=measure(x)
            return abs(gap+.025)+err*2+max(own.values(),default=0.)*2000+max(support.values(),default=0.)*1500+max(0.,pen-.8)*500+np.linalg.norm((x[:4]-d.qpos[active])/TICK)*.001
        bounds=list(zip(low[active],high[active]))+[(-24.,24.) if args.allow_upper_contact else (0.,22.),(10.,24.)];rng=np.random.default_rng(831+index);population=rng.uniform(np.array(bounds)[:,0],np.array(bounds)[:,1],(72,6));population[0,:4]=np.clip(d.qpos[active],low[active],high[active]);population[0,4:]=[10.,15.]
        fit=differential_evolution(objective,bounds,seed=831+index,init=population,maxiter=150,tol=1e-7,polish=True,workers=1);gap,err,support,own,pen,point=measure(fit.x);contacts=[]
        for c in ik.contact:
            if {int(c.geom1),int(c.geom2)}!={cube,tip} or c.dist>=.0002:continue
            n=c.frame[:3]*(1 if int(c.geom1)==cube else -1);lever=float(np.cross(c.pos-C,-n)@up)
            contacts.append(dict(point_cube_frame_mm=((c.pos-C)@B*1000).tolist(),normal_alignment=float(n@normal),positive_yaw_moment_arm_mm=lever*1000,distance_mm=float(c.dist*1000)))
        goal=motor(ik.ctrl);valid=bool(any(c['normal_alignment']>.7 and c['positive_yaw_moment_arm_mm']>3 and np.linalg.norm(np.asarray(c['point_cube_frame_mm'])-point)<8 for c in contacts) and max(own.values(),default=0.)<=1e-6 and max(support.values(),default=0.)<=.15 and pen<=1 and np.all(goal>=g.lo) and np.all(goal<=g.hi))
        name=f'axis{axis}_sign{sign:+d}';path=folder/(name+'_proposal.npz');np.savez_compressed(path,**{k:getattr(ik,k) for k in BOUND},time=ik.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        row=dict(profile=name,seed=831+index,finger=args.finger,nfev=int(fit.nfev),objective=float(fit.fun),target_point_cube_frame_mm=point.tolist(),point_plane_gap_mm=gap,normal_error=err,contacts=contacts,self_pairs_mm=own,support_pairs_mm=support,cube_penetration_mm=pen,static_contact_candidate=valid,proposal=path.relative_to(OUT).as_posix(),proposal_sha256=sha(path));rows.append(row);save_json(folder/'progress.json',dict(rows=rows));print(json.dumps({k:row[k] for k in ('profile','objective','static_contact_candidate','contacts')}),flush=True)
    save_json(folder/'review.json',dict(status='declared_finger_positive_yaw_contact_geometry',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),model_provenance=provenance,sliding_friction_hypothesis=args.sliding_friction,rows=rows,only_declared_finger_changed_in_IK=args.finger,temporary_upper_contact_allowed=args.allow_upper_contact,cube_fixed_only_in_proposal_copies=True,original_guards_and_bounds_preserved=True,positive_yaw_lever_is_geometry_not_dynamic_success=True,free_object_arrival_not_evaluated=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
