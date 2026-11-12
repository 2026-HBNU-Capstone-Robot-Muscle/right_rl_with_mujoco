"""Choose an easy common grip endpoint; no physical execution is admitted."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import json,time,argparse
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
R=Path(__file__).resolve().parent;CP=R/'right_hand_rotation_20261105/preintegration/reference_end';O=R/'right_hand_rotation_20261105/canonical_body_grip/easy_endpoint_search'
A=np.r_[np.arange(4,8),np.arange(12,16)]

def unit(v):return v/np.linalg.norm(v)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--margin',type=float,default=.7);ap.add_argument('--level-only',action='store_true');ap.add_argument('--tag');a=ap.parse_args();out=O if not a.tag else O/a.tag;out.mkdir(parents=True,exist_ok=True)
    m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb'));z=np.load(CP/'end_state.npz');d=restore(m,z);base=d.qpos.copy();pd=d.ctrl-base[:16];cube=m.geom('cube').id;C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy();up=np.array(z['up']);hand,_,check=inspect_for(m);geoms=precompute(m,hand)
    forward=unit(B0[:,2]-up*(B0[:,2]@up));side=unit(np.cross(up,forward));basis=np.column_stack([up,forward,side]);y=-up;zz=unit(B0[:,2]-y*(B0[:,2]@y));level=np.column_stack([unit(np.cross(y,zz)),y,zz]);lo,hi,nlo,nhi=native(m);ids=np.array(SIM_TO_MOTOR_IDS)-1;low=np.maximum((lo[ids]+4-2048)*TICK-pd,nlo);high=np.minimum((hi[ids]-4-2048)*TICK-pd,nhi);tips=[]
    for name in ('mf_tip','th_tip'):
        g=m.geom(name).id;c=min((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,g}),key=lambda c:c.dist);G=d.geom_xmat[g].reshape(3,3);n=c.frame[:3]*(1 if int(c.geom1)==cube else -1);cl=B0.T@(c.pos-C0);tips.append((g,G.T@(c.pos-d.geom_xpos[g]),G.T@n,1 if cl[0]>0 else -1,cl[2]*1000))
    lower=np.r_[low[A],0,-20,-20,-24,-24];upper=np.r_[high[A],40,20,20,24,24];results=[]
    seedstate=np.load(R/'right_hand_rotation_20261105/preintegration/body_grip/lower_m20_t16_n10/end_state.npz');seedq=np.asarray(seedstate['ctrl'])-pd
    for label,B in ((('level',level),) if a.level_only else (('same_tilt',B0),('level',level))):
      for lift in ((12.,25.,38.) if a.level_only else (0.,12.,25.,38.)):
        seed=seedq[A] if lift>0 else base[A];x0=np.r_[np.clip(seed,low[A]+1e-5,high[A]-1e-5),lift,0,0,[np.clip(t[4],-23.9,23.9) for t in tips]];began=time.perf_counter()
        def measure(x,exact=False):
            C=C0+basis@x[8:11]/1000;d.qpos[:]=base;d.qpos[A]=x[:8];quat=__import__('scipy').spatial.transform.Rotation.from_matrix(B).as_quat();d.qpos[-7:]=np.r_[C,quat[3],quat[:3]];d.ctrl[:]=d.qpos[:16]+pd;d.qvel[:]=0;mujoco.mj_forward(m,d);e=[];n=[]
            for i,(g,local,ln,sign,tan) in enumerate(tips):
                target=np.array([sign*.0272,.012,x[11+i]/1000]);G=d.geom_xmat[g].reshape(3,3);e.extend((d.geom_xpos[g]+G@local-(C+B@target))*1000);n.extend((G@ln-B[:,0]*sign)*3)
            clearance=gaps(m,d,C,B,geoms,exact=exact);su,sp=check(d);pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            return np.array(e),np.array(n),clearance,max(sp.values(),default=0.),max(su.values(),default=0.),pen,C
        def residual(x):
            e,n,cl,su,sp,pen,C=measure(x)
            return np.r_[e,n,np.maximum(a.margin-np.array([v for name,v in cl]),0)*8,(x[:8]-base[A])*.05,x[8:11]*.025,su*500,sp*500,max(0,pen-.8)*100]
        fit=least_squares(residual,x0,bounds=(lower,upper),max_nfev=90,ftol=1e-7,xtol=1e-7,gtol=1e-7);e,n,cl,su,sp,pen,C=measure(fit.x,True);err=float(max(np.linalg.norm(e.reshape(2,3),axis=1)));margin=min(v for name,v in cl);normal_angles=[float(np.degrees(np.arccos(np.clip(1-(v@v)/18,-1,1)))) for v in n.reshape(2,3)];accepted=bool(err<.8 and margin>.3 and su<=1e-6 and sp<=.15 and pen<=1 and max(normal_angles)<35)
        row=dict(orientation=label,seed_lift_mm=lift,tip_error_mm=err,normal_error_deg=normal_angles,minimum_full_sweep_gap_mm=margin,self_mm=su,support_mm=sp,cube_mm=pen,goal=motor(d.ctrl).tolist(),cube_center_world=C.tolist(),cube_rotation_world=B.tolist(),cube_center_offset_mm=fit.x[8:11].tolist(),endpoint_only_accepted=accepted,elapsed_seconds=time.perf_counter()-began,closest_geoms=sorted(cl,key=lambda v:v[1])[:8]);results.append(row)
        (out/'review.json').write_text(json.dumps(dict(status='common_terminal_endpoint_only_research',requested_clearance_margin_mm=a.margin,cases=results,physical_or_dynamic_success_verified=False,pinned_cube_for_IK_only=True,geometry_force_and_native_bounds_unchanged=True,hardware_execution_allowed=False,devices_opened=False),indent=2)+'\n',encoding='utf-8');print(json.dumps(row),flush=True)
        if accepted:np.savez_compressed(out/f'endpoint_{label}_{lift:g}.npz',goal=motor(d.ctrl),cube_center=C,cube_rotation=B);break
if __name__=='__main__':main()
