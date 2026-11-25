"""Endpoint proposals with whole finger envelopes outside the upper sweep.

The cube is fixed for IK only. No path or physical execution is admitted here.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import json,time
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from audit_upper_layer_clearance_20261104 import local_mesh
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/preintegration/clearance_ik';CP=O.parent/'reference_end'
ACTIVE=np.r_[np.arange(4,8),np.arange(12,16)]

def main():
    O.mkdir(parents=True,exist_ok=True);m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb'));z=np.load(CP/'end_state.npz');d=restore(m,z);base=d.qpos.copy();pd=d.ctrl-base[:16];cube=m.geom('cube').id;C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();_,_,check=inspect_for(m)
    lo,hi,_,_=native(m);from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK
    ids=np.array(SIM_TO_MOTOR_IDS)-1;low=(lo[ids]+4-2048)*TICK-pd;high=(hi[ids]-4-2048)*TICK-pd;tips=[]
    for name in ('mf_tip','th_tip'):
        g=m.geom(name).id;c=min((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,g}),key=lambda c:c.dist);G=d.geom_xmat[g].reshape(3,3);normal=c.frame[:3]*(1 if int(c.geom1)==cube else -1)
        cl=B.T@(c.pos-C);tips.append((g,G.T@(c.pos-d.geom_xpos[g]),G.T@normal,cl))
    geoms=[]
    for g in range(m.ngeom):
        name=m.geom(g).name or ''
        if not(name.startswith(('mf_','th_')) or '_mf_' in name or '_th_' in name) or not(m.geom_contype[g] or m.geom_conaffinity[g]):continue
        if int(m.geom_type[g])==7:
            mesh=int(m.geom_dataid[g]);a=int(m.mesh_vertadr[mesh]);v=np.asarray(m.mesh_vert[a:a+int(m.mesh_vertnum[mesh])],float)
        elif int(m.geom_type[g])==6:v=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*m.geom_size[g]
        else:continue
        geoms.append((g,(v.max(0)+v.min(0))/2,(v.max(0)-v.min(0))/2))
    def measurements(x,height):
        d.qpos[:]=base;d.qpos[ACTIVE]=x[:8];d.ctrl[:]=d.qpos[:16]+pd;d.qvel[:]=0;mujoco.mj_forward(m,d);errors=[];normals=[]
        for i,(g,local,ln,cl) in enumerate(tips):
            target=cl.copy();target[1]=height/1000;target[2]=x[8+i]/1000
            G=d.geom_xmat[g].reshape(3,3);errors.extend((d.geom_xpos[g]+G@local-(C+B@target))*1000);normals.extend((G@ln-B[:,0]*(1 if cl[0]>0 else -1))*3)
        gaps=[]
        for g,lc,lh in geoms:
            A=B.T@d.geom_xmat[g].reshape(3,3);cc=B.T@(d.geom_xpos[g]-C)+A@lc;hh=np.abs(A)@lh;delta=cc*1000-np.array([0.,-55/3,0.]);radii=hh*1000+np.array([27.5*np.sqrt(2),55/6,27.5*np.sqrt(2)]);gaps.append(float(max(abs(delta)-radii)))
        su,sp=check(d);pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
        return np.array(errors),np.array(normals),np.array(gaps),max(sp.values(),default=0.),max(su.values(),default=0.),pen
    results=[];rng=np.random.default_rng(115)
    for height in (4.,10.,16.):
      for attempt in range(4):
        seed=base[ACTIVE] if attempt==0 else np.clip(base[ACTIVE]+rng.normal(0,.4,8),low[ACTIVE]+1e-5,high[ACTIVE]-1e-5)
        x0=np.r_[seed,[np.clip(cl[2]*1000,-20,20) for g,l,n,cl in tips]];began=time.perf_counter()
        def residual(x):
            e,n,g,su,sp,pen=measurements(x,height)
            return np.r_[e,n,np.maximum(.5-g,0)*12,(x[:8]-base[ACTIVE])*.1,su*500,sp*500,max(0,pen-.8)*100]
        fit=least_squares(residual,x0,bounds=(np.r_[low[ACTIVE],-20,-20],np.r_[high[ACTIVE],20,20]),max_nfev=130,ftol=1e-7,xtol=1e-7,gtol=1e-7)
        e,n,g,su,sp,pen=measurements(fit.x,height);err=float(max(np.linalg.norm(e.reshape(2,3),axis=1)));clear=float(min(g));good=bool(err<.8 and clear>.3 and su<=1e-6 and sp<=.15 and pen<=1)
        row=dict(height_mm=height,attempt=attempt,tip_error_mm=err,minimum_conservative_gap_mm=clear,self_mm=su,support_mm=sp,cube_mm=pen,endpoint_proposal_accepted=good,goal=motor(d.ctrl).tolist(),x=fit.x.tolist(),elapsed_seconds=time.perf_counter()-began)
        results.append(row);(O/'review.json').write_text(json.dumps(dict(status='geometric_endpoint_search_only',cases=results,hardware_execution_allowed=False,devices_opened=False,pinned_cube_for_ik_only=True,dynamic_path_not_validated=True),indent=2)+'\n',encoding='utf-8');print(json.dumps(row),flush=True)
        if good:np.savez_compressed(O/f'endpoint_h{height:g}_a{attempt}.npz',goal=motor(d.ctrl));break
if __name__=='__main__':main()
