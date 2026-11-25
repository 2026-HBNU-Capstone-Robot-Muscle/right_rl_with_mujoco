"""Find opposite lower-face contacts while searching a nearby cube station.

Cube translation in IK is hypothetical. No proposal is called a reached grip;
unchanged free-object dynamics must be tested separately. Original station and
its failed trials remain intact. This script never accesses hardware.
"""
from pathlib import Path
import argparse
import json
import time
import numpy as np
import mujoco
from scipy.optimize import differential_evolution
from scipy.spatial import ConvexHull
from native_cycle import OUT, assets, restore, Guard, BOUND, motor, TICK, SIM_TO_MOTOR_IDS, save_json, sha
from upper_layer_cylinder_clearance_20261105 import precompute, gaps
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--profile', choices=('X','Z'), default='X')
    parser.add_argument('--seed', type=int, default=121)
    parser.add_argument('--iterations', type=int, default=240)
    parser.add_argument('--fixed-station',action='store_true')
    parser.add_argument('--free-contact-point',action='store_true')
    args = parser.parse_args()
    base = OUT/'next_cycle/handoff_refinement_20261110'
    folder = base/f'opposing_workspace_{args.profile}_seed{args.seed}'
    if args.fixed_station:folder=folder.with_name(folder.name+'_fixed')
    if args.free_contact_point:folder=folder.with_name(folder.name+'_surface')
    if folder.exists(): raise RuntimeError('Preserve prior workspace study')
    source = base/'first90_RF_shape_before_adduction/RF_park_MF_then_RF_shape_then_adduct/phase0_end_state.npz'
    model, initial, _, _, provenance = assets('active_ccd50')
    z = dict(np.load(source, allow_pickle=False)); data = restore(model,z)
    guard = Guard(model); cube = model.geom('cube').id
    C0 = data.geom_xpos[cube].copy(); B = data.geom_xmat[cube].reshape(3,3).copy()
    lag = data.ctrl-data.qpos[:16]; sid = np.asarray(SIM_TO_MOTOR_IDS)-1
    low = np.maximum(guard.qlo,(np.ceil(guard.lo[sid])+2-2048)*TICK-lag)
    high = np.minimum(guard.qhi,(np.floor(guard.hi[sid])-2-2048)*TICK-lag)
    active = np.r_[np.arange(4,8),np.arange(12,16)]
    axis = 0 if args.profile=='X' else 2; across = 2 if axis==0 else 0
    sign = 1 if args.profile=='X' else -1
    geometries = precompute(model,guard.hand); vertices = {g:v for g,v,_ in geometries}
    fingers=[]
    for name,group,direction in (('mf',np.arange(4,8),sign),('th',np.arange(12,16),-sign)):
        tip=model.geom(name+'_tip').id; hull=ConvexHull(vertices[tip])
        point=np.zeros(3); point[axis]=direction*27.45; point[1]=10.; point[across]=15.
        fingers.append((name,group,tip,hull.equations[:,:3],-hull.equations[:,3],point,direction*B[:,axis]))
    proposal=restore(model,z)
    def measure(x, expensive=False):
        proposal.qpos[:]=z['qpos']; proposal.qpos[active]=x[:8]
        proposal.qpos[-7:-4]=C0+x[8:11]/1000
        proposal.ctrl[:]=proposal.qpos[:16]+lag; mujoco.mj_forward(model,proposal)
        C=proposal.geom_xpos[cube]; fit=[]
        for name,group,tip,A,b,point,normal in fingers:
            if args.free_contact_point:
                point=point.copy();point[1]=x[11];point[across]=x[12]
            R=proposal.geom_xmat[tip].reshape(3,3)
            local=R.T@(C+B@point/1000-proposal.geom_xpos[tip]); planes=A@local-b; j=int(np.argmax(planes))
            fit.append((float(planes[j]*1000),float(np.linalg.norm(R@A[j]+normal))))
        support,own=guard.contacts(proposal)
        pen=max((-float(c.dist)*1000 for c in proposal.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
        upper=min((row[1] for row in gaps(model,proposal,C,B,geometries,True)),default=100.) if expensive else None
        return fit,max(own.values(),default=0.),max(support.values(),default=0.),pen,upper
    def objective(x):
        fit,own,support,pen,_=measure(x)
        return sum(abs(gap+.04)+normal*3. for gap,normal in fit)+own*2000+support*1500+max(0.,pen-.8)*700+np.linalg.norm(x[8:11])*.015
    limits=list(zip(low[active],high[active]))+([(0.,0.)]*3 if args.fixed_station else [(-15.,15.)]*3)
    if args.free_contact_point:limits += [(0.,23.),(-24.,24.)]
    rng=np.random.default_rng(args.seed); population=rng.uniform(np.array(limits)[:,0],np.array(limits)[:,1],(110,len(limits)))
    population[0,:8]=np.clip(data.qpos[active],low[active],high[active]); population[0,8:]=0
    if args.free_contact_point:population[0,11:]=[10.,15.]
    old=base/'opposing_actual_cube_contact_geometry'
    for row,name in enumerate(('mf','th')):
        p=old/f'opposed_{args.profile}_{name}_proposal.npz'
        if p.exists():
            q=np.load(p,allow_pickle=False)['qpos']; population[0,row*4:row*4+4]=np.clip(q[active[row*4:row*4+4]],low[active[row*4:row*4+4]],high[active[row*4:row*4+4]])
    for index in range(1,10):
        population[index]=population[0]
        if not args.fixed_station:population[index,8:11]=rng.uniform(-12,12,3)
    folder.mkdir();(folder/'source_search_opposing_grip_workspace.py').write_bytes(Path(__file__).read_bytes())
    began=time.perf_counter(); generations=[]
    def callback(x, convergence):
        value=objective(x); generations.append(value)
        if len(generations)%30==0:
            save_json(folder/'progress.json',dict(generation=len(generations),objective=value,cube_shift_mm=x[8:11].tolist()))
            print(json.dumps(dict(profile=args.profile,generation=len(generations),objective=value)),flush=True)
    result=differential_evolution(objective,limits,init=population,seed=args.seed,maxiter=args.iterations,callback=callback,tol=1e-7,polish=True,workers=1)
    fit,own,support,pen,upper=measure(result.x,True); C=proposal.geom_xpos[cube].copy()
    contact_rows=[]; fingers_valid=[]
    for name,group,tip,A,b,point,normal in fingers:
        if args.free_contact_point:
            point=point.copy();point[1]=result.x[11];point[across]=result.x[12]
        rows=[]
        for c in proposal.contact:
            if {int(c.geom1),int(c.geom2)}!={cube,tip} or c.dist>=.0002:continue
            n=c.frame[:3]*(1 if int(c.geom1)==cube else -1)
            rows.append(dict(point_cube_frame_mm=((c.pos-C)@B*1000).tolist(),normal_alignment=float(n@normal),distance_mm=float(c.dist*1000)))
        fingers_valid.append(any(r['normal_alignment']>.8 and np.linalg.norm(np.array(r['point_cube_frame_mm'])-point)<8 for r in rows))
        contact_rows.append(dict(finger=name,target_cube_frame_mm=point.tolist(),contacts=rows))
    cert=SweepCertificate(model,guard.hand,.3); lower,bad=cert.query(proposal,C,B)
    goal=motor(proposal.ctrl)
    candidate=bool(all(fingers_valid) and own<=1e-6 and support<=.15 and pen<=1 and np.all(goal>=guard.lo) and np.all(goal<=guard.hi))
    np.savez_compressed(folder/'proposal.npz',**{k:getattr(proposal,k) for k in BOUND},time=proposal.time,
        up=initial['up'],initial_cube_rotation=initial['initial_cube_rotation'],initial_cube_center=initial['initial_cube_center'])
    report=dict(status='nearby_opposite_face_workspace_geometry',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),model_provenance=provenance,
        profile=args.profile,seed=args.seed,nfev=int(result.nfev),objective=float(result.fun),center=C.tolist(),cube_shift_mm=result.x[8:11].tolist(),
        station_fixed_at_actual_checkpoint=args.fixed_station,contact_point_optimized_on_lower_face=args.free_contact_point,
        joint_degrees=np.degrees(proposal.qpos[:16]).tolist(),point_plane_gaps_and_normal_errors=fit,contacts=contact_rows,self_mm=own,support_mm=support,cube_penetration_mm=pen,
        static_opposed_contact_candidate=candidate,upper_fast_screen_mm=upper,upper_certificate_lower_mm=lower,bad_upper_geoms=[model.geom(int(cert.ids[i])).name for i in bad],
        proposal=(folder/'proposal.npz').relative_to(OUT).as_posix(),proposal_sha256=sha(folder/'proposal.npz'),
        translation_is_hypothetical_IK_not_executed=True,old_canonical_target_and_failures_preserved=True,arrival_path_passed=False,free_object_hold_passed=False,
        finite_search_not_general_infeasibility_proof=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began)
    save_json(folder/'review.json',report)
    print(json.dumps({k:report[k] for k in ('static_opposed_contact_candidate','cube_shift_mm','point_plane_gaps_and_normal_errors','self_mm','cube_penetration_mm','upper_certificate_lower_mm','elapsed_seconds')}),flush=True)


if __name__=='__main__':main()
