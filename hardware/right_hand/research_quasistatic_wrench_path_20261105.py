"""Position goals from bounded contact-force allocation in isolated simulation.

Uses perfect simulated pose/contact data and a quasistatic PD model. It does
not write contact forces into the engine. The cube moves only through bounded
right-hand position goals; the stricter .15/.23Nm source model is preserved.
No hardware policy, camera or motor adapter is provided.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from scipy.optimize import minimize,Bounds,LinearConstraint,nnls,lsq_linear
from scipy.spatial.transform import Rotation
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor,angle
from hand_motor_mapping import TICK,SIM_TO_MOTOR_IDS,ticks_to_sim
from cube_task_frame_20261105 import orientation_error_deg,upright_yaw_target
from canonical_grip_dynamics_20261105 import run
from native_pyramidal_point_forces_20261106 import point_rays
from offline_fractional_goal_accumulator_20261105 import FractionalGoals
from checkpoint_quarter_review_20261106 import quarter_reasons
from offline_goal_tracking_bounds_20261106 import tracking_increment_bounds

def hybrid_increment(m,d,allocation,planner,C,targetC,B,targetB,tip_memory=None,contact_height_targets=None,world_up=None,maximum_tracking_rad=None,ignored_motion_geoms=(),contact_normal_frames=None,tip_memory_names=('mf_tip','th_tip'),recovery_normal_targets=None):
    active=np.r_[np.arange(8),np.arange(12,16)];ids=np.asarray(SIM_TO_MOTOR_IDS)-1;K=m.actuator_gainprm[:,0]
    translation=(targetC-C)*.12;length=np.linalg.norm(translation)
    if length>.0003:translation*=.0003/length
    omega=Rotation.from_matrix(targetB@B.T).as_rotvec()*.12;length=np.linalg.norm(omega)
    if length>.0075:omega*=.0075/length
    matrix=[];rhs=[];tasks=list(allocation['contact_tasks'])
    if tip_memory is not None:
        for name in tip_memory_names:
            g=m.geom(name).id;G=d.geom_xmat[g].reshape(3,3);origin=d.geom_xpos[g]
            present=[t for t in tasks if t['geom_id']==g and t['actual_normal_N']>.05]
            if present:
                t=max(present,key=lambda t:t['actual_normal_N']);p=np.asarray(t['point']);n=np.asarray(t['outward_normal'])
                tip_memory[name]=(G.T@(p-origin),B.T@(p-C),B.T@n)
            elif name in tip_memory and not any(t['geom_id']==g for t in tasks):
                local,clocal,cn=tip_memory[name];point=origin+G@local;target=C+B@clocal;error=target-point;norm=np.linalg.norm(error)
                if norm>.002:error*=.002/norm
                desired_normal=1.5 if recovery_normal_targets is None else float(recovery_normal_targets[name])
                tasks.append(dict(geom_id=g,point=point,outward_normal=B@cn,actual_normal_N=0.,predicted_normal_N=desired_normal,recover_tip_displacement=error))
    for task in tasks:
        g=task['geom_id']
        if m.geom(g).name in ignored_motion_geoms:continue
        point=np.asarray(task['point']);normal=np.asarray(task['outward_normal']);jp=np.zeros((3,m.nv));jr=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,jr,point,int(m.geom_bodyid[g]));J=jp[:,active];row=normal@J;desired=translation+np.cross(omega,point-C);singular=np.linalg.svd(J,compute_uv=False);rank=int(sum(singular>1e-8))
        desired+=.12*np.asarray(task.get('recover_tip_displacement',[0.,0.,0.]))
        name=m.geom(g).name
        if contact_height_targets is not None and name in contact_height_targets and rank==3:
            up=np.asarray(world_up,float);tangent_up=up-normal*float(normal@up);height=float((point-C)@up);slide=float(np.clip(.15*(contact_height_targets[name]-height),-.00012,.00012));desired+=tangent_up*slide
        compliance=float(np.sum(row**2/K[active]));error=task['predicted_normal_N']-task['actual_normal_N'];closing=0. if rank<3 else float(np.clip(error*compliance*.05,-.00008,.00008))
        matrix.append(row[None,:]*10000);rhs.append(np.array([normal@desired-closing])*10000)
        if rank==3:
            tangent=np.eye(3)-np.outer(normal,normal);matrix.append(tangent@J*2000);rhs.append(tangent@desired*2000)
            rotation=np.column_stack([np.cross(jr[:,j],normal) for j in active]);matrix.append(rotation*5);rhs.append(np.cross(omega,normal)*5)
    # A contact normal changing to the top face must not silently redefine the
    # intended lower-side grasp. These optional frames retain the source's
    # hand-local and cube-local normals; no model force or pose is injected.
    if contact_normal_frames is not None:
        for name,(tip_local,cube_local) in contact_normal_frames.items():
            g=m.geom(name).id;jr=np.zeros((3,m.nv));mujoco.mj_jac(m,d,None,jr,d.geom_xpos[g],int(m.geom_bodyid[g]));normal=d.geom_xmat[g].reshape(3,3)@tip_local;target_normal=B@cube_local;rotation=np.column_stack([np.cross(jr[:,j],normal) for j in active]);matrix.append(rotation*30);rhs.append((np.cross(omega,normal)+.12*(target_normal-normal))*30)
    matrix.append(np.eye(len(active))*.02);rhs.append(np.zeros(len(active)));A=np.vstack(matrix);v=np.concatenate(rhs);lower,upper=planner.increment_bounds()
    if maximum_tracking_rad is not None:
        actual=np.empty(16);actual[ids]=d.qpos[:16]/TICK+2048;lower,upper=tracking_increment_bounds(lower,upper,planner.continuous,actual,float(maximum_tracking_rad)/TICK)
    lower=lower[ids[active]]*TICK;upper=upper[ids[active]]*TICK;fixed=upper-lower<1e-12;change=np.zeros(len(active));change[fixed]=(lower[fixed]+upper[fixed])/2;free=~fixed
    if np.any(free):change[free]=lsq_linear(A[:,free],v-A[:,fixed]@change[fixed],bounds=(lower[free],upper[free]),tol=1e-10,max_iter=100).x
    increment=np.zeros(16);increment[ids[active]]=change/TICK;return planner.advance(increment)

def allocate_native(m,d,cube,C,F,T,normal_reference,hand=None,include_unloaded=False,limits_cache=None,preferred_contact_names=('mf_tip','th_tip'),normal_reference_weight=.2):
    if int(m.opt.cone)!=int(mujoco.mjtCone.mjCONE_PYRAMIDAL):return None,dict(reason='native_pyramid_model_required')
    K=m.actuator_gainprm[:,0]
    if limits_cache is None:_,_,lo,hi=native(m)
    else:lo,hi=limits_cache
    if hand is None:hand,_,_=inspect_for(m)
    blocks=[];jac=[];edges=[];physical_seed=[];seed=[];groups={};tasks=[]
    for ci,c in enumerate(d.contact):
        a,b=int(c.geom1),int(c.geom2)
        if cube not in (a,b):continue
        g=b if a==cube else a
        if int(m.geom_bodyid[g]) not in hand:continue
        force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force)
        if (force[0]<1e-5 and not include_unloaded) or float(c.dist)>=.0004:continue
        if int(c.dim)!=3:return None,dict(reason='unsupported_native_contact_dimension')
        frame=c.frame.reshape(3,3);normal=frame[0]*(1 if a==cube else -1);basis=np.column_stack([-normal,frame[1],frame[2]]);E=point_rays(float(c.friction[0]),float(c.friction[1]));world=basis@E
        actual=frame.T@force[:3]*(-1 if a==cube else 1);coordinates=basis.T@actual;initial,_=nnls(E,coordinates);seed.extend(initial);physical_seed.extend(coordinates);edges.append(E)
        tasks.append(dict(geom_id=g,point=c.pos.tolist(),outward_normal=normal.tolist(),actual_normal_N=float(force[0])))
        jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,c.pos,int(m.geom_bodyid[g]));r=c.pos-C;blocks.append(np.vstack([world,np.column_stack([np.cross(r,world[:,j]) for j in range(4)])]));jac.append(jp[:,:16].T@world);groups.setdefault(m.geom(g).name,[]).append(len(edges)-1)
    if len(edges)<(1 if include_unloaded else 3):return None,dict(reason='too_few_actual_contact_points',contacts=len(edges))
    W=np.column_stack(blocks);J=np.column_stack(jac);seed=np.asarray(seed);P=np.zeros((3*len(edges),4*len(edges)))
    for i,E in enumerate(edges):P[3*i:3*i+3,4*i:4*i+4]=E
    bias=d.qfrc_bias[:16]-d.qfrc_passive[:16];enabled=np.asarray(m.actuator_forcelimited,bool);flo=np.where(enabled,m.actuator_forcerange[:,0],-np.inf);fhi=np.where(enabled,m.actuator_forcerange[:,1],np.inf)
    lower=np.maximum(np.maximum(K*(lo-d.qpos[:16]),-.18*K),flo)-bias;upper=np.minimum(np.minimum(K*(hi-d.qpos[:16]),.18*K),fhi)-bias
    G=np.zeros((len(groups),len(seed)));targets=[];weights=[]
    for row,(name,indices) in zip(G,groups.items()):
        for i in indices:row[4*i:4*i+4]=edges[i][0]
        current=float(row@seed);normal_reference.setdefault(name,current);targets.append(normal_reference[name]);weights.append(1. if name in preferred_contact_names else 0.)
    scale=np.r_[np.full(3,10.),np.full(3,300.)];weights=np.asarray(weights);A=np.vstack([W*scale[:,None],P*.05,G*weights[:,None]*normal_reference_weight,np.eye(len(seed))*1e-4]);target=np.r_[np.r_[F,T]*scale,np.asarray(physical_seed)*.05,np.asarray(targets)*weights*normal_reference_weight,np.zeros(len(seed))];H=A.T@A;v=A.T@target
    fit=minimize(lambda x:float(.5*x@H@x-v@x),seed,jac=lambda x:H@x-v,method='SLSQP',bounds=Bounds(np.zeros(len(seed)),np.full(len(seed),3.5)),constraints=[LinearConstraint(J,lower,upper),LinearConstraint(G,0,3.5)],options=dict(maxiter=200,ftol=1e-10))
    x=fit.x;violation=max(0,float(max(lower-J@x)),float(max(J@x-upper)),float(max(G@x-3.5)),-float(min(x)));res=W@x-np.r_[F,T]
    if not fit.success or violation>1e-6:return None,dict(reason='bounded_native_force_optimizer_failed',message=fit.message,violation=violation,wrench_residual=res.tolist())
    proposed=d.qpos[:16]+(bias+J@x)/K;proposed[8:12]=d.ctrl[8:12]
    for i,task in enumerate(tasks):task['predicted_normal_N']=float(edges[i][0]@x[4*i:4*i+4])
    return proposed,dict(contact_points=len(edges),contact_tasks=tasks,native_friction_rays=True,unloaded_engine_generated_contact_points_included=include_unloaded,normal_N={name:float(row@x) for name,row in zip(groups,G)},desired_wrench_tracking_residual=res.tolist(),wrench_tracking_is_objective_not_certificate=True)

def allocate(m,d,cube,C,F,T,normal_reference):
    K=m.actuator_gainprm[:,0];_,_,lo,hi=native(m);hand,_,_=inspect_for(m);blocks=[];jac=[];seed=[];mu=[];groups={}
    for ci,c in enumerate(d.contact):
        a,b=int(c.geom1),int(c.geom2)
        if cube not in (a,b):continue
        g=b if a==cube else a
        if int(m.geom_bodyid[g]) not in hand:continue
        force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force)
        if force[0]<1e-5:continue
        frame=c.frame.reshape(3,3);normal=frame[0]*(1 if a==cube else -1);basis=np.column_stack([-normal,frame[1],frame[2]])
        actual=frame.T@force[:3]*(-1 if a==cube else 1);coords=basis.T@actual
        jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,c.pos,int(m.geom_bodyid[g]));r=c.pos-C
        blocks.append(np.vstack([basis,np.column_stack([np.cross(r,basis[:,j]) for j in range(3)])]));jac.append(jp[:,:16].T@basis);seed.extend(coords);mu.append(float(c.friction[0]));groups.setdefault(m.geom(g).name,[]).append(len(mu)-1)
    if len(mu)<3:return None,dict(reason='too_few_actual_contact_points',contacts=len(mu))
    W=np.column_stack(blocks);J=np.column_stack(jac);seed=np.asarray(seed);bias=d.qfrc_bias[:16]-d.qfrc_passive[:16]
    enabled=np.asarray(m.actuator_forcelimited,bool);flo=np.where(enabled,m.actuator_forcerange[:,0],-np.inf);fhi=np.where(enabled,m.actuator_forcerange[:,1],np.inf)
    lower=np.maximum(np.maximum(K*(lo-d.qpos[:16]),-.18*K),flo)-bias;upper=np.minimum(np.minimum(K*(hi-d.qpos[:16]),.18*K),fhi)-bias
    G=np.zeros((len(groups),len(seed)));targets=[];weights=[]
    for row,(name,indices) in zip(G,groups.items()):
        for i in indices:row[3*i]=1
        current=float(row@seed);normal_reference.setdefault(name,current);targets.append(normal_reference[name]);weights.append(1. if name in ('mf_tip','th_tip') else 0.)
    targets=np.asarray(targets);weights=np.asarray(weights)
    for i,value in enumerate(mu):
        limit=value*seed[3*i];length=np.linalg.norm(seed[3*i+1:3*i+3])
        if length>limit:seed[3*i+1:3*i+3]*=limit/length
    def cones(x):return np.array([value*x[3*i]-np.linalg.norm(x[3*i+1:3*i+3]) for i,value in enumerate(mu)])
    def cost(x):return float(.5*np.sum((x-seed)**2)+np.sum(weights*(G@x-targets)**2))
    target=np.r_[F,T];fit=minimize(cost,seed,method='SLSQP',bounds=Bounds(np.tile([0.,-3.5,-3.5],len(mu)),np.full(len(seed),3.5)),constraints=[LinearConstraint(J,lower,upper),LinearConstraint(G,0,3.5),LinearConstraint(W,target,target),{'type':'ineq','fun':cones}],options=dict(maxiter=200,ftol=1e-9))
    x=fit.x;res=W@x-target;violation=max(0,float(max(lower-J@x)),float(max(J@x-upper)),float(max(G@x-3.5)),-float(min(cones(x))))
    if not fit.success or max(abs(res))>1e-5 or violation>1e-6:return None,dict(reason='static_force_allocation_not_certified',message=fit.message,wrench_residual=res.tolist(),violation=violation)
    proposed=d.qpos[:16]+(bias+J@x)/K;proposed[8:12]=d.ctrl[8:12]
    return proposed,dict(contact_points=len(mu),normal_N={name:float(row@x) for name,row in zip(groups,G)},wrench_error=float(max(abs(res))))

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--task',choices=('hold','quarter'),required=True);ap.add_argument('--cap',type=float,choices=(.15,.23),default=.15);ap.add_argument('--samples',type=int,default=600);ap.add_argument('--revision',choices=('legacy','native_cone'),default='native_cone');ap.add_argument('--include-unloaded-contacts',action='store_true');ap.add_argument('--hybrid-motion',action='store_true');ap.add_argument('--lift-mm',type=float,choices=(4.,6.,8.),default=8.);ap.add_argument('--phase-orientation-deg',type=float,choices=(3.,5.),default=3.);ap.add_argument('--planning-margin-ticks',type=int,choices=(0,8,16),default=0)
    ap.add_argument('--retain-tip-tracking',action='store_true');ap.add_argument('--upright-final',action='store_true');ap.add_argument('--gravity-upright',action='store_true');ap.add_argument('--target-yaw-deg',type=float,choices=(90.,94.,96.,98.),default=90.);ap.add_argument('--station-y-mm',type=float,choices=(0.,4.,8.,12.),default=0.);a=ap.parse_args()
    if a.gravity_upright and a.upright_final:raise ValueError('select one target construction')
    if not 134<=a.samples<=900:raise ValueError('bounded offline workload')
    folder=ROOT/f'torque_cap_sensitivity/cap{a.cap:g}';source=folder/'end_state.npz';model=folder/'model.mjb';z=np.load(source);m=mujoco.MjModel.from_binary_path(str(model));d=restore(m,z)
    revised=a.revision=='native_cone'
    if a.hybrid_motion and not revised:raise ValueError('hybrid transport uses the verified native cone')
    variant=(f'_lift{a.lift_mm:g}' if a.lift_mm!=8 else '')+(f'_phase{a.phase_orientation_deg:g}' if a.phase_orientation_deg!=3 else '')+(f'_margin{a.planning_margin_ticks}' if a.planning_margin_ticks else '')
    variant+='_retained_tips' if a.retain_tip_tracking else ''
    variant+='_upright' if a.upright_final else ''
    variant+='_gravity_upright' if a.gravity_upright else ''
    variant+=f'_yaw{a.target_yaw_deg:g}' if a.target_yaw_deg!=90 else ''
    variant+=f'_stationY{a.station_y_mm:g}' if a.station_y_mm else ''
    out=ROOT/f'quasistatic_wrench_path/{a.task}_cap{a.cap:g}{"_native_cone" if revised else ""}{"_potential_contacts" if a.include_unloaded_contacts else ""}{"_hybrid_motion" if a.hybrid_motion else ""}{variant}';out.mkdir(parents=True,exist_ok=True)
    cube=m.geom('cube').id;body=m.body('cube').id;up=z['up'];C0=d.qpos[-7:-4].copy();B0=d.xmat[body].reshape(3,3).copy();_,_,nlo,nhi=native(m);lo,hi,_,_=native(m);hand,support,check=inspect_for(m)
    station_delta=np.array([0.,a.station_y_mm/1000,0.]);station_center=C0+station_delta
    initial_yaw=angle(B0,z['initial_cube_rotation'],up);finalB=Rotation.from_rotvec(up*np.radians(a.target_yaw_deg-initial_yaw)).as_matrix()@B0;goal=motor(d.ctrl);start=goal.copy();ids=np.array(SIM_TO_MOTOR_IDS)-1
    if a.upright_final:finalB=Rotation.from_rotvec(up*np.radians(a.target_yaw_deg)).as_matrix()@z['initial_cube_rotation']
    if a.gravity_upright:finalB=upright_yaw_target(z['initial_cube_rotation'],up,a.target_yaw_deg)
    turn_vector=Rotation.from_matrix(finalB@B0.T).as_rotvec()
    # Tighten planning only. Preserve a previously legal source goal when it
    # already lies near a boundary; it may retreat but never move farther out.
    planlo=np.minimum(lo+a.planning_margin_ticks,start);planhi=np.maximum(hi-a.planning_margin_ticks,start)
    planner=FractionalGoals(start,planlo,planhi);tip_memory={} if a.retain_tip_tracking else None
    phase=('regulate' if revised else 'hold') if a.task=='hold' else 'lift';phase_start=0;stable=0;held=0;rows=[];phases=[];trace=[];snapshots=[];normal_reference={};stop=None;search_stop=None;gap=longest=0.;allocation={};began=time.perf_counter()
    worst=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,range_rad=0.);properties={k:getattr(m,k).copy() for k in ('actuator_gainprm','actuator_biasprm','actuator_forcelimited','actuator_forcerange','jnt_range','geom_size','geom_contype','geom_conaffinity')}
    for index in range(1,a.samples+1):
        C=d.qpos[-7:-4].copy();B=d.xmat[body].reshape(3,3);elapsed=(index-1-phase_start)*.075
        blend=float(np.clip(elapsed/3,0,1));blend=blend*blend*(3-2*blend)
        targetC=C0.copy();targetB=B0
        if phase=='lift':targetC=C0+(station_delta+up*(a.lift_mm/1000))*blend
        if phase=='yaw':targetC=station_center+up*(a.lift_mm/1000);targetB=Rotation.from_rotvec(turn_vector*min(1,elapsed/6)).as_matrix()@B0
        if phase=='lower':targetC=station_center+up*(a.lift_mm/1000)*(1-blend);targetB=finalB
        if phase=='hold' and a.task=='quarter':targetC=station_center;targetB=finalB
        if phase=='hold':held+=1
        if phase!='hold' or (a.task=='hold' and not revised):
            velocity=np.zeros(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,body,velocity,0)
            F=-m.body_mass[body]*m.opt.gravity+30*(targetC-C)-1.5*velocity[3:]
            T=.04*Rotation.from_matrix(targetB@B.T).as_rotvec()-.002*velocity[:3]
            if revised:proposed,allocation=allocate_native(m,d,cube,C,F,T,normal_reference,hand,a.include_unloaded_contacts,(nlo,nhi))
            else:proposed,allocation=allocate(m,d,cube,C,F,T,normal_reference)
            if proposed is None:search_stop=allocation['reason'];break
            if a.hybrid_motion:new=hybrid_increment(m,d,allocation,planner,C,targetC,B,targetB,tip_memory)
            else:
                target=(proposed/TICK+2048);raw=np.empty(16);raw[ids]=target
                if np.any(raw<lo-1e-7) or np.any(raw>hi+1e-7):search_stop='native_goal_proposal_rejected';break
                delta=np.clip(raw-goal,-2.,2.);new=np.rint(goal+delta).astype(int)
            if max(abs(new-goal))>2:raise RuntimeError('goal increment bound')
            goal=new
        d.ctrl[:]=ticks_to_sim(goal);rows.append(goal.copy());phases.append(phase)
        for sub in range(75):
            mujoco.mj_step(m,d);su,sp=check(d);worst['self_mm']=max(worst['self_mm'],max(sp.values(),default=0.));worst['support_mm']=max(worst['support_mm'],max(su.values(),default=0.));worst['tracking_rad']=max(worst['tracking_rad'],float(max(abs(d.qpos[:16]-d.ctrl))));worst['range_rad']=max(worst['range_rad'],float(max(0,max(nlo-d.qpos[:16]),max(d.qpos[:16]-nhi))))
            inhand=external=False
            for c in d.contact:
                x,y=int(c.geom1),int(c.geom2)
                if cube not in (x,y):continue
                other=y if x==cube else x
                if int(m.geom_bodyid[other]) in hand:inhand|=c.dist<.0004;worst['cube_mm']=max(worst['cube_mm'],max(0.,-float(c.dist)*1000))
                elif other in support and c.dist<.0004:external=True
            gap=0. if inhand else gap+.001;longest=max(longest,gap)
            if (not np.isfinite(np.r_[d.qpos,d.qvel]).all() or worst['self_mm']>1e-6 or worst['support_mm']>.15 or worst['cube_mm']>1 or worst['tracking_rad']>.18 or worst['range_rad']>1e-6 or external or longest>.25 or np.linalg.norm(d.qpos[-7:-4]-C0)>.08):stop='original dynamic bound';break
        C=d.qpos[-7:-4].copy();B=d.xmat[body].reshape(3,3);position_error=float(np.linalg.norm(C-targetC)*1000);rotation_error=orientation_error_deg(B,targetB);tilt=float(np.degrees(np.arccos(np.clip((-B[:,1])@up,-1,1))));yaw=angle(B,z['initial_cube_rotation'],up)
        if position_error>15 or rotation_error>20:search_stop='task_pose_divergence';break
        if phase=='regulate' and index>=67:phase='hold';phase_start=index
        elif phase in ('lift','yaw','lower'):
            end_time=6 if phase=='yaw' else 3;stable=stable+1 if elapsed>=end_time and position_error<1 and rotation_error<a.phase_orientation_deg and tilt<5 else 0
            if stable>=20:phase={'lift':'yaw','yaw':'lower','lower':'hold'}[phase];phase_start=index;stable=0
        if index%20==0 or stop or held>=134:trace.append(dict(sample=index,phase=phase,yaw_deg=yaw,tilt_deg=tilt,lift_mm=float((C-C0)@up*1000),position_error_mm=position_error,rotation_error_deg=rotation_error,allocation={k:v for k,v in allocation.items() if k!='contact_tasks'}));print(json.dumps(trace[-1]),flush=True)
        if index%2==1 or stop or held>=134:snapshots.append(d.qpos.copy())
        if stop or held>=134:break
    for k,v in properties.items():assert np.array_equal(getattr(m,k),v)
    fresh_lo,fresh_hi,fresh_nlo,fresh_nhi=native(m)
    assert all(np.array_equal(x,y) for x,y in ((lo,fresh_lo),(hi,fresh_hi),(nlo,fresh_nlo),(nhi,fresh_nhi)))
    np.savez_compressed(out/'suffix.npz',start=start,goals=rows,phases=phases);np.savez_compressed(out/'trajectory.npz',qpos=snapshots)
    np.savez_compressed(out/'end_state.npz',qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,act=d.act,qacc_warmstart=d.qacc_warmstart,time=d.time,up=up,initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
    verification=None
    constant_tail=bool(len(rows)>=134 and np.array_equal(np.asarray(rows[-134:]),np.tile(rows[-1],(134,1))))
    if stop is None and search_stop is None and held>=134 and constant_tail:verification=run(m,z,rows,1,out/'verified_end_state.npz',out/'verified_trajectory.npz',True)
    result=dict(status='quasistatic_force_allocation_to_bounded_position_goals',task=a.task,hypothetical_model_torque_cap_Nm=a.cap,source_state=str(source.relative_to(R)),source_state_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),source_model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),samples=len(rows),final_phase=phase,fixed_hold_samples=held,guard_stop=stop,search_stop=search_stop,allocation_at_exit=allocation,final_yaw_deg=angle(d.xmat[body].reshape(3,3),z['initial_cube_rotation'],up),final_center_shift_mm=float(np.linalg.norm(d.qpos[-7:-4]-C0)*1000),worst=worst,trace=trace,recorded_goal_replay_verification=verification,perfect_simulated_contact_and_pose_feedback=True,force_allocation_is_quasistatic_hypothesis=True,no_external_cube_force_or_pose_written=True,ring_targets_held_fixed=True,current_PID_and_original_dynamic_bounds_not_changed=True,full_initial_placement_not_replayed_for_this_feedback_path=True,hardware_execution_allowed=False,devices_opened=False,elapsed_seconds=time.perf_counter()-began)
    result.update(revision=a.revision,include_unloaded_engine_contacts=a.include_unloaded_contacts,hybrid_geometric_motion_and_force_preload=a.hybrid_motion,last134_goals_proved_constant=constant_tail,legacy_adaptive_hold_does_not_prove_fixed_hold=not revised,native_dynamic_guards_not_relaxed=True)
    result['allocation_at_exit']={k:v for k,v in allocation.items() if k!='contact_tasks'}
    result['controller_source_sha256']=hashlib.sha256(__import__('pathlib').Path(__file__).read_bytes()).hexdigest()
    result['retain_lost_tip_tracking']=a.retain_tip_tracking;result['upright_final_orientation']=a.upright_final
    result['intermediate_target_yaw_deg']=a.target_yaw_deg
    result['gravity_aligned_final_orientation']=a.gravity_upright
    result['legacy_upright_label_only_rotates_initial_reference_and_can_preserve_its_tip']=a.upright_final
    result['linear_algebra_thread_environment']={k:os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')}
    reasons=quarter_reasons(verification,station_center,constant_tail,orientation_error_deg(d.xmat[body].reshape(3,3),finalB)) if a.task=='quarter' else None
    result.update(planned_new_station_center_world_m=station_center.tolist(),planned_hand_Y_translation_mm=a.station_y_mm,previous_common_center_not_claimed=bool(a.station_y_mm))
    result.update(temporary_lift_target_mm=a.lift_mm,intermediate_phase_orientation_tolerance_deg=a.phase_orientation_deg,planning_native_margin_ticks=a.planning_margin_ticks,checkpoint_quarter_filter_reasons=reasons,checkpoint_quarter_filter_passed=False if reasons is None else not reasons,full_initial_placement_quarter_not_yet_certified=True)
    (out/'review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in result.items() if k not in ('trace','recorded_goal_replay_verification')}),flush=True)
if __name__=='__main__':main()
