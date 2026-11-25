"""Geometry proposals with a bounded optimizer seed; dynamic state untouched."""
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from research_rotation_body_hold_20261104 import restore,native,motor,inspect_for
from hand_motor_mapping import TICK,SIM_TO_MOTOR_IDS
A=np.r_[np.arange(4,8),np.arange(12,16)]

def propose_bounded(m,z,turn,weight):
    d=restore(m,z);p=restore(m,z);cube=m.geom('cube').id;C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();up=z['up'];base=d.qpos.copy();pd=d.ctrl-base[:16];lo,hi,nlo,nhi=native(m);ids=np.array(SIM_TO_MOTOR_IDS)-1
    low=np.maximum((np.ceil(lo[ids])+4-2048)*TICK-pd,nlo);high=np.minimum((np.floor(hi[ids])-4-2048)*TICK-pd,nhi);hand,_,check=inspect_for(m);tips=[]
    for name in ('mf_tip','th_tip'):
        g=m.geom(name).id;c=min((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,g}),key=lambda c:c.dist);G=d.geom_xmat[g].reshape(3,3);n=c.frame[:3]*(1 if int(c.geom1)==cube else -1);tips.append((g,G.T@(c.pos-d.geom_xpos[g]),G.T@n,B.T@(c.pos-C),B.T@n))
    current=base[:16].copy();previous=motor(d.ctrl);rows=[];records=[];count=max(12,int(np.ceil(abs(turn)/.5)))
    for step in range(1,count+1):
        Br=Rotation.from_rotvec(up*np.radians(turn*step/count)).as_matrix()@B;q=Rotation.from_matrix(Br).as_quat();pose=np.r_[C,q[3],q[:3]];seed=current[A].copy();lower=np.maximum(low[A],seed-.15);upper=np.minimum(high[A],seed+.15)
        if np.any(lower>=upper):return None,records,'no valid native IK interval'
        fitseed=np.clip(seed,lower+1e-9,upper-1e-9)
        def measure(x):
            p.qpos[:]=base;p.qpos[:16]=current;p.qpos[A]=x;p.qpos[-7:]=pose;p.ctrl[:]=p.qpos[:16]+pd;p.qvel[:]=0;mujoco.mj_forward(m,p);e=[];n=[]
            for g,local,ln,cl,cn in tips:
                G=p.geom_xmat[g].reshape(3,3);e.extend((p.geom_xpos[g]+G@local-(C+Br@cl))*1000);n.extend((G@ln-Br@cn)*weight)
            su,sp=check(p);pen=max((max(0.,-float(c.dist)*1000) for c in p.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            return np.array(e),np.array(n),max(sp.values(),default=0.),max(su.values(),default=0.),pen
        def residual(x):
            e,n,su,sp,pen=measure(x);return np.r_[e,n,(x-seed)*.3,su*500,sp*500,max(0,pen-.8)*100]
        fit=least_squares(residual,fitseed,bounds=(lower,upper),max_nfev=100,ftol=1e-8,xtol=1e-8,gtol=1e-8);e,n,su,sp,pen=measure(fit.x);err=float(max(np.linalg.norm(e.reshape(2,3),axis=1)));goal=motor(p.ctrl)
        records.append(dict(step=step,tip_error_mm=err,self_mm=su,support_mm=sp,cube_mm=pen,optimizer_seed_adjustment_ticks=float(max(abs(fitseed-seed))/TICK),goal=goal.tolist()))
        if err>.8 or su>1e-6 or sp>.15 or pen>1 or np.any(goal<lo) or np.any(goal>hi):return None,records,'unchanged geometry/native bound'
        current[A]=fit.x;nsteps=max(1,int(np.ceil(max(abs(goal-previous))/2)));rows.extend(np.rint(previous+(goal-previous)*j/nsteps).astype(int) for j in range(1,nsteps+1));previous=goal
    rows.extend(previous.copy() for _ in range(134))
    return np.array(rows),records,None
