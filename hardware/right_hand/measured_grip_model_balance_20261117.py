"""Native-friction wrench allocation for offline contact preload only."""
import numpy as np
import mujoco
from scipy.optimize import linprog
from native_pyramidal_point_forces_20261106 import point_rays


def balance(m,d,hand,cube,nlo,nhi):
    blocks=[];jac=[];groups={};rays=[]
    center=d.geom_xpos[cube]
    for c in d.contact:
        a,b=int(c.geom1),int(c.geom2)
        if cube not in (a,b):continue
        geom=b if a==cube else a;body=int(m.geom_bodyid[geom])
        if body not in hand or c.dist>=.0004:continue
        if int(c.dim)!=3:return None,{'reason':'unsupported contact dimension'}
        normal=c.frame[:3]*(1 if a==cube else -1)
        E=point_rays(float(c.friction[0]),float(c.friction[1]))
        world=np.column_stack([-normal,c.frame[3:6],c.frame[6:9]])@E
        radius=c.pos-center
        blocks.append(np.vstack([world,np.column_stack([np.cross(radius,world[:,i]) for i in range(4)])]))
        jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,c.pos,body)
        jac.append(jp[:,:16].T@world);rays.append(E)
        role=(m.body(body).name or '').split('_')[0]
        groups.setdefault(role,[]).append(len(rays)-1)
    if not all(k in groups for k in ('palm','if','th')):return None,{'reason':'contact groups unavailable'}
    W=np.column_stack(blocks);J=np.column_stack(jac);K=m.actuator_gainprm[:,0]
    bias=d.qfrc_bias[:16]-d.qfrc_passive[:16]
    lower=np.maximum(np.maximum(K*(nlo-d.qpos[:16]),-.12*K),m.actuator_forcerange[:,0])-bias
    upper=np.minimum(np.minimum(K*(nhi-d.qpos[:16]),.12*K),m.actuator_forcerange[:,1])-bias
    G=np.zeros((len(groups),W.shape[1]))
    for row,indices in zip(G,groups.values()):
        for index in indices:row[4*index:4*index+4]=rays[index][0]
    required=G[[list(groups).index(k) for k in ('if','th','palm')]]
    limits=np.array([1.0 if k in ('if','th') else 3.5 for k in groups])
    A=np.vstack([J,-J,G,-required]);b=np.r_[upper,-lower,limits,[-.65,-.65,-.2]]
    mass=m.body_mass[m.body('cube').id];wanted=np.r_[-mass*m.opt.gravity,np.zeros(3)]
    fit=linprog(np.ones(W.shape[1]),A_ub=A,b_ub=b,A_eq=W,b_eq=wanted,
                bounds=[(0,3.5)]*W.shape[1],method='highs')
    if not fit.success:return None,{'reason':'native wrench allocation infeasible'}
    control=d.qpos[:16]+(bias+J@fit.x)/K
    control[4:12]=d.ctrl[4:12]
    return control,dict(group_normal_n={k:float(v) for k,v in zip(groups,G@fit.x)},
        wrench_error=float(max(abs(W@fit.x-wanted))),physical_current_conversion=False)
