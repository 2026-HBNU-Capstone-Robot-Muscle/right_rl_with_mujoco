"""Keep current body contact anchors while moving obstructing housings.

Pinned-cube proposals only; free-cube replay is a separate mandatory check.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,json,time
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from research_rotation_body_hold_20261104 import restore,native,motor,inspect_for
from hand_motor_mapping import TICK,SIM_TO_MOTOR_IDS
from dynamic_common_grip_search_20261105 import CP,ROOT,R
from upper_layer_cylinder_clearance_20261105 import precompute,gaps

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--normal',type=float,default=5);ap.add_argument('--margin',type=float,default=2);ap.add_argument('--exact',action='store_true');ap.add_argument('--tag',required=True);a=ap.parse_args()
    source=ROOT/'coordinated_refinement/turn18_normal10/end_state.npz';out=ROOT/'contact_preserving_endpoint'/a.tag;out.mkdir(parents=True,exist_ok=True);m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb'));z=np.load(source);d=restore(m,z);base=d.qpos.copy();pd=d.ctrl-base[:16];cube=m.geom('cube').id;C0=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();up=z['up'];forward=B[:,2]-up*(B[:,2]@up);forward/=np.linalg.norm(forward);side=np.cross(up,forward);basis=np.column_stack([up,forward,side]);quat=Rotation.from_matrix(B).as_quat();lo,hi,nlo,nhi=native(m);ids=np.array(SIM_TO_MOTOR_IDS)-1;low=np.maximum((np.ceil(lo[ids])+4-2048)*TICK-pd,nlo);high=np.minimum((np.floor(hi[ids])-4-2048)*TICK-pd,nhi);A=np.r_[0,np.arange(4,8),np.arange(12,16)];K=len(A);hand,_,check=inspect_for(m);geoms=precompute(m,hand);anchors=[]
    for name in ('mf_tip','th_tip','if_bs_collision_1'):
        g=m.geom(name).id;cs=[c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,g}]
        if not cs:raise ValueError('missing source contact '+name)
        c=min(cs,key=lambda c:c.dist);G=d.geom_xmat[g].reshape(3,3);n=c.frame[:3]*(1 if int(c.geom1)==cube else -1);anchors.append((g,G.T@(c.pos-d.geom_xpos[g]),G.T@n,B.T@(c.pos-C0),B.T@n))
    lower=np.r_[low[A],-2,-20,-20];upper=np.r_[high[A],30,20,20];x0=np.r_[np.clip(base[A],low[A]+1e-8,high[A]-1e-8),0,0,0]
    def measure(x,exact=False):
        C=C0+basis@x[K:]/1000;d.qpos[:]=base;d.qpos[A]=x[:K];d.qpos[-7:]=np.r_[C,quat[3],quat[:3]];d.ctrl[:]=d.qpos[:16]+pd;d.qvel[:]=0;mujoco.mj_forward(m,d);e=[];n=[]
        for i,(g,local,ln,cl,cn) in enumerate(anchors):
            G=d.geom_xmat[g].reshape(3,3);e.extend((d.geom_xpos[g]+G@local-(C+B@cl))*1000*(1 if i<2 else .5))
            if i<2:n.extend((G@ln-B@cn)*a.normal)
        clear=gaps(m,d,C,B,geoms,exact);su,sp=check(d);pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
        return np.array(e),np.array(n),clear,max(sp.values(),default=0.),max(su.values(),default=0.),pen,C
    def residual(x):
        e,n,clear,selfmm,supportmm,pen,C=measure(x,a.exact)
        return np.r_[e,n,np.maximum(a.margin-np.array([v for name,v in clear]),0)*10,(x[:K]-base[A])*.1,x[K:]*.03,selfmm*500,supportmm*500,max(0,pen-.8)*100]
    began=time.perf_counter();fit=least_squares(residual,x0,bounds=(lower,upper),max_nfev=180,ftol=1e-8,xtol=1e-8,gtol=1e-8);e,n,cl,selfmm,supportmm,pen,C=measure(fit.x,True);err=float(max(np.linalg.norm(e.reshape(3,3),axis=1)));gap=min(v for name,v in cl);accepted=bool(err<.8 and gap>.3 and selfmm<=1e-6 and supportmm<=.15 and pen<=1)
    report=dict(status='existing_contact_preserving_clearance_endpoint_only',source=str(source.relative_to(R)),parameters=vars(a),anchor_error_mm=err,minimum_full_sweep_gap_mm=gap,self_mm=selfmm,support_mm=supportmm,cube_mm=pen,goal=motor(d.ctrl).tolist(),cube_translation_mm=fit.x[K:].tolist(),closest_geoms=sorted(cl,key=lambda r:r[1])[:6],endpoint_only_accepted=accepted,elapsed_seconds=time.perf_counter()-began,pinned_cube_for_proposal_only=True,hardware_execution_allowed=False,devices_opened=False)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    if accepted:np.savez_compressed(out/'endpoint.npz',goal=motor(d.ctrl),cube_center=C,cube_rotation=B)
    print(json.dumps(report),flush=True)
if __name__=='__main__':main()
