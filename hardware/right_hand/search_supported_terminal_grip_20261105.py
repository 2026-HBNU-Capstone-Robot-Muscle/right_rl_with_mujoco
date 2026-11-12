"""Search a common terminal grasp with an actual index support contact.

Static geometric proposals only. No devices, no dynamic success claim.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,json,time
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
R=Path(__file__).resolve().parent;CP=R/'right_hand_rotation_20261105/preintegration/reference_end';O=R/'right_hand_rotation_20261105/canonical_body_grip/supported_endpoint'
A=np.r_[0,np.arange(4,8),np.arange(12,16)];K=len(A)
def unit(x):return x/np.linalg.norm(x)
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--margin',type=float,default=3);ap.add_argument('--seed-lift',type=float,default=20);ap.add_argument('--seed-root',type=int,default=256);ap.add_argument('--yaw',type=float,default=0);a=ap.parse_args();out=O/f'lift{a.seed_lift:g}_root{a.seed_root}_margin{a.margin:g}_yaw{a.yaw:g}';out.mkdir(parents=True,exist_ok=True)
    m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb'));z=np.load(CP/'end_state.npz');d=restore(m,z);base=d.qpos.copy();pd=d.ctrl-base[:16];cube=m.geom('cube').id;C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy();up=z['up'];forward=unit(B0[:,2]-up*(B0[:,2]@up));side=unit(np.cross(up,forward));basis=np.column_stack([up,forward,side]);y=-up;B=np.column_stack([unit(np.cross(y,forward)),y,forward]);quat=Rotation.from_matrix(B).as_quat();hand,_,check=inspect_for(m);geoms=precompute(m,hand);lo,hi,nlo,nhi=native(m);ids=np.array(SIM_TO_MOTOR_IDS)-1;low=np.maximum((lo[ids]+4-2048)*TICK-pd,nlo);high=np.minimum((hi[ids]-4-2048)*TICK-pd,nhi)
    if not -30<=a.yaw<=30:raise ValueError('bounded offline orientation search')
    B=Rotation.from_rotvec(up*np.radians(a.yaw)).as_matrix()@B;quat=Rotation.from_matrix(B).as_quat()
    tips=[]
    for name in ('mf_tip','th_tip','if_bs_collision_1'):
        g=m.geom(name).id;c=min((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,g}),key=lambda c:c.dist);G=d.geom_xmat[g].reshape(3,3);n=c.frame[:3]*(1 if int(c.geom1)==cube else -1);cl=B0.T@(c.pos-C0);tips.append((g,G.T@(c.pos-d.geom_xpos[g]),G.T@n,cl,1 if cl[0]>0 else -1))
    ep=np.load(R/'right_hand_rotation_20261105/canonical_body_grip/easy_endpoint_search/margin3_native/endpoint_level_25.npz');seed=(ep['goal'][ids]-2048)*TICK-pd;seed[0]=base[0]+a.seed_root*TICK
    lowx=np.r_[low[A],0,-25,-25,-24,-24,-24,-24];highx=np.r_[high[A],40,25,25,24,24,24,24]
    x0=np.r_[np.clip(seed[A],low[A]+1e-5,high[A]-1e-5),a.seed_lift,0,0,[np.clip(tips[i][3][2]*1000,-23.9,23.9) for i in range(2)],np.clip(tips[2][3][[0,2]]*1000,-23.9,23.9)]
    def measure(x,exact=False):
        C=C0+basis@x[K:K+3]/1000;d.qpos[:]=base;d.qpos[A]=x[:K];d.qpos[-7:]=np.r_[C,quat[3],quat[:3]];d.ctrl[:]=d.qpos[:16]+pd;d.qvel[:]=0;mujoco.mj_forward(m,d);e=[];n=[]
        for i,(g,local,ln,cl,sign) in enumerate(tips):
            target=np.array([sign*.0272,.012,x[K+3+i]/1000]) if i<2 else np.array([x[K+5]/1000,.0272,x[K+6]/1000])
            G=d.geom_xmat[g].reshape(3,3);e.extend((d.geom_xpos[g]+G@local-(C+B@target))*1000)
            if i<2:n.extend((G@ln-B[:,0]*sign)*3)
        cl=gaps(m,d,C,B,geoms,exact);su,sp=check(d);pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
        return np.array(e),np.array(n),cl,max(sp.values(),default=0.),max(su.values(),default=0.),pen,C
    def residual(x):
        e,n,cl,selfmm,supportmm,pen,C=measure(x)
        return np.r_[e,n,np.maximum(a.margin-np.array([v for name,v in cl]),0)*8,(x[:K]-base[A])*.05,x[K:K+3]*.025,selfmm*500,supportmm*500,max(0,pen-.8)*100]
    began=time.perf_counter();fit=least_squares(residual,x0,bounds=(lowx,highx),max_nfev=160,ftol=1e-7,xtol=1e-7,gtol=1e-7);e,n,cl,selfmm,supportmm,pen,C=measure(fit.x,True);err=float(max(np.linalg.norm(e.reshape(3,3),axis=1)));gap=min(v for name,v in cl);accepted=bool(err<.8 and gap>.3 and selfmm<=1e-6 and supportmm<=.15 and pen<=1)
    report=dict(status='supported_common_grip_endpoint_only',parameters=vars(a),tip_and_support_error_mm=err,minimum_full_sweep_gap_mm=gap,self_mm=selfmm,support_mm=supportmm,cube_mm=pen,goal=motor(d.ctrl).tolist(),cube_center_offset_mm=fit.x[K:K+3].tolist(),closest_geoms=sorted(cl,key=lambda r:r[1])[:6],endpoint_only_accepted=accepted,elapsed_seconds=time.perf_counter()-began,cube_pinned_for_geometry_only=True,dynamic_or_physical_success_verified=False,hardware_execution_allowed=False,devices_opened=False)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    if accepted:np.savez_compressed(out/'endpoint.npz',goal=motor(d.ctrl),cube_center=C,cube_rotation=B)
    print(json.dumps(report),flush=True)
if __name__=='__main__':main()
