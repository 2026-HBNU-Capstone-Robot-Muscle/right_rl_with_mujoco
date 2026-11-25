"""Exact native-friction wrench feasibility for offline supported grasps."""
import numpy as np,mujoco
from scipy.optimize import linprog
from native_pyramidal_point_forces_20261106 import point_rays
from research_rotation_body_hold_20261104 import motor


def allocate(m,d,cube,hand,nlo,nhi,lo,hi,tip_minimum=.6,required_tips=('if_tip','th_tip'),normal_only=False,normal_bounds=None):
    if int(m.opt.cone)!=int(mujoco.mjtCone.mjCONE_PYRAMIDAL):
        return None,dict(reason='native_pyramidal_model_required')
    center=d.geom_xpos[cube];body=m.body('cube').id;B=d.geom_xmat[cube].reshape(3,3)
    blocks=[];jac=[];groups={};rays=[];underside=[]
    for c in d.contact:
        a,b=int(c.geom1),int(c.geom2)
        if cube not in (a,b):continue
        geom=b if a==cube else a
        if int(m.geom_bodyid[geom]) not in hand or c.dist>=.0004:continue
        if int(c.dim)!=3:return None,dict(reason='unsupported_contact_dimension')
        normal=c.frame[:3]*(1 if a==cube else -1)
        E=point_rays(float(c.friction[0]),float(c.friction[1]))
        world=np.column_stack([-normal,c.frame[3:6],c.frame[6:9]])@E
        r=c.pos-center
        blocks.append(np.vstack([world,np.column_stack([np.cross(r,world[:,i]) for i in range(4)])]))
        jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,c.pos,int(m.geom_bodyid[geom]))
        jac.append(jp[:,:16].T@world);rays.append(E)
        groups.setdefault(m.geom(geom).name,[]).append(len(rays)-1)
        if float(r@B[:,1])>.020 and float(normal@B[:,1])>.5:underside.append(len(rays)-1)
    required=tuple(required_tips)
    if len(required)<2 or len(set(required))!=len(required):
        raise ValueError('Distinct opposed finger contact names required')
    if not all(n in groups for n in required) or not underside:
        return None,dict(reason='missing_opposed_or_underside_contacts')
    W=np.column_stack(blocks);J=np.column_stack(jac);count=W.shape[1]
    K=m.actuator_gainprm[:,0];bias=d.qfrc_bias[:16]-d.qfrc_passive[:16]
    enabled=np.asarray(m.actuator_forcelimited,bool)
    flo=np.where(enabled,m.actuator_forcerange[:,0],-np.inf);fhi=np.where(enabled,m.actuator_forcerange[:,1],np.inf)
    lower=np.maximum(np.maximum(K*(nlo-d.qpos[:16]),-.18*K),flo)-bias
    upper=np.minimum(np.minimum(K*(nhi-d.qpos[:16]),.18*K),fhi)-bias
    G=np.zeros((len(groups),count))
    for row,indices in zip(G,groups.values()):
        for index in indices:row[4*index:4*index+4]=rays[index][0]
    tip_rows=G[[list(groups).index(n) for n in required]]
    under=np.zeros(count)
    for index in underside:under[4*index:4*index+4]=rays[index][0]
    A=np.vstack([J,-J,G,-tip_rows,-under[None,:]])
    bounds=np.r_[upper,-lower,np.full(len(groups),3.5),[-tip_minimum]*len(required),[-.2]]
    if normal_bounds:
        extra=[];limits=[]
        for name,(minimum,maximum) in normal_bounds.items():
            if not np.isfinite([minimum,maximum]).all() or not 0<=minimum<=maximum<=3.5:
                raise ValueError('Invalid explicit contact-normal bounds')
            if name not in groups:
                if minimum>0:return None,dict(reason='missing_bounded_contact',name=name)
                continue
            row=G[list(groups).index(name)]
            extra.extend([row,-row]);limits.extend([maximum,-minimum])
        if extra:A=np.vstack([A,extra]);bounds=np.r_[bounds,limits]
    target=np.r_[-m.body_mass[body]*m.opt.gravity,np.zeros(3)]
    # Equality constrains the actual native contact-force rays, rather than
    # accepting a weighted approximation to gravity/torque balance.
    equality=W;wanted=target
    if normal_only:
        tangent=np.zeros((2*len(rays),count))
        for index,E in enumerate(rays):tangent[2*index:2*index+2,4*index:4*index+4]=E[1:]
        equality=np.vstack([W,tangent]);wanted=np.r_[target,np.zeros(2*len(rays))]
    fit=linprog(np.ones(count),A_ub=A,b_ub=bounds,A_eq=equality,b_eq=wanted,
                bounds=[(0,3.5)]*count,method='highs')
    if not fit.success:return None,dict(reason='exact_native_wrench_infeasible',message=fit.message)
    x=fit.x;residual=W@x-target;violation=max(0.,float(np.max(A@x-bounds)))
    if max(abs(residual))>1e-7 or violation>1e-7:
        return None,dict(reason='exact_native_wrench_residual',residual=residual.tolist(),violation=violation)
    control=d.qpos[:16]+(bias+J@x)/K;control[8:12]=d.ctrl[8:12]
    goal=motor(control)
    if np.any(goal<lo) or np.any(goal>hi):return None,dict(reason='quantized_native_target_rejected')
    return control,dict(contact_points=len(rays),exact_wrench_residual=residual.tolist(),
        constraint_violation=violation,normal_N={n:float(v) for n,v in zip(groups,G@x)},
        underside_normal_N=float(under@x),tip_minimum_N=tip_minimum,native_contact_model_unchanged=True,
        normal_only_force_solution=normal_only,explicit_normal_bounds=normal_bounds)
