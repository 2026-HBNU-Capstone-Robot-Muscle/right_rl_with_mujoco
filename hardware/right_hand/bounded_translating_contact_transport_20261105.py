"""IK proposals allowing temporary cube translation; never a dynamic proof."""
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from research_rotation_body_hold_20261104 import restore,native,motor,inspect_for
from hand_motor_mapping import TICK,SIM_TO_MOTOR_IDS
A=np.r_[np.arange(4,8),np.arange(12,16)]

def propose_translating(m,z,turn,weight,self_weight=500.):
    d=restore(m,z);p=restore(m,z);cube=m.geom('cube').id;C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy();up=z['up'];base=d.qpos.copy();pd=d.ctrl-base[:16];lo,hi,nlo,nhi=native(m);ids=np.array(SIM_TO_MOTOR_IDS)-1
    low=np.maximum((np.ceil(lo[ids])+4-2048)*TICK-pd,nlo);high=np.minimum((np.floor(hi[ids])-4-2048)*TICK-pd,nhi);_,_,check=inspect_for(m);tips=[]
    for name in ('mf_tip','th_tip'):
        gid=m.geom(name).id;c=min((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,gid}),key=lambda c:c.dist);G=d.geom_xmat[gid].reshape(3,3);normal=c.frame[:3]*(1 if int(c.geom1)==cube else -1);tips.append((gid,G.T@(c.pos-d.geom_xpos[gid]),G.T@normal,B0.T@(c.pos-C0),B0.T@normal))
    current=base[:16].copy();translation=np.zeros(3);previous=motor(d.ctrl);rows=[];records=[];count=max(12,int(np.ceil(abs(turn)/.5)))
    for step in range(1,count+1):
        Br=Rotation.from_rotvec(up*np.radians(turn*step/count)).as_matrix()@B0;quat=Rotation.from_matrix(Br).as_quat();seed=np.r_[current[A],translation];lower=np.r_[np.maximum(low[A],current[A]-.15),np.maximum(-20,translation-2)];upper=np.r_[np.minimum(high[A],current[A]+.15),np.minimum(20,translation+2)]
        if np.any(lower>=upper):return None,records,'no valid native/translation interval'
        fitseed=np.clip(seed,lower+1e-9,upper-1e-9)
        def measure(x):
            C=C0+x[-3:]/1000;p.qpos[:]=base;p.qpos[:16]=current;p.qpos[A]=x[:len(A)];p.qpos[-7:]=np.r_[C,quat[3],quat[:3]];p.ctrl[:]=p.qpos[:16]+pd;p.qvel[:]=0;mujoco.mj_forward(m,p);errors=[];normal_errors=[]
            for gid,local,normal,anchor,cn in tips:
                G=p.geom_xmat[gid].reshape(3,3);errors.extend((p.geom_xpos[gid]+G@local-(C+Br@anchor))*1000);normal_errors.extend((G@normal-Br@cn)*weight)
            su,sp=check(p);pen=max((max(0,-float(c.dist)*1000) for c in p.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            return np.array(errors),np.array(normal_errors),max(sp.values(),default=0.),max(su.values(),default=0.),pen
        def residual(x):
            error,normals,selfmm,supportmm,pen=measure(x)
            return np.r_[error,normals,(x[:len(A)]-seed[:len(A)])*.3,(x[-3:]-translation)*.03,x[-3:]*.01,selfmm*self_weight,supportmm*500,max(0,pen-.8)*100]
        fit=least_squares(residual,fitseed,bounds=(lower,upper),max_nfev=140,ftol=1e-8,xtol=1e-8,gtol=1e-8);error,normals,selfmm,supportmm,pen=measure(fit.x);err=float(max(np.linalg.norm(error.reshape(2,3),axis=1)));goal=motor(p.ctrl)
        records.append(dict(step=step,tip_error_mm=err,self_mm=selfmm,support_mm=supportmm,cube_mm=pen,translation_mm=fit.x[-3:].tolist(),goal=goal.tolist()))
        if err>.8 or selfmm>1e-6 or supportmm>.15 or pen>1 or np.any(goal<lo) or np.any(goal>hi):return None,records,'unchanged geometry/native bound'
        current[A]=fit.x[:len(A)];translation=fit.x[-3:].copy();nsteps=max(1,int(np.ceil(max(abs(goal-previous))/2)));rows.extend(np.rint(previous+(goal-previous)*j/nsteps).astype(int) for j in range(1,nsteps+1));previous=goal
    rows.extend(previous.copy() for _ in range(134))
    return np.array(rows),records,None
