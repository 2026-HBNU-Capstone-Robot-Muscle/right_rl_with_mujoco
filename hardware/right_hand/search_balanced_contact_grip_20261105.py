"""Geometric common grip using actual CAD contacts, not a stale tip anchor.

The cube is pinned for proposals only. No dynamic or physical approval is made.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,json,time
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares,minimize
from scipy.spatial.transform import Rotation
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from upper_layer_cylinder_clearance_20261105 import precompute,gaps

R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/canonical_body_grip';CP=O.parent/'preintegration/reference_end'
A=np.r_[0,np.arange(4,8),np.arange(12,16)];K=len(A)

def unit(v):return v/np.linalg.norm(v)

def contact_geometry(m,d,cube,g,C,B):
    cs=[c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,g}]
    if cs:
        depth=min(float(c.dist) for c in cs);near=[c for c in cs if c.dist<=depth+.0001]
        point=np.mean([c.pos for c in near],axis=0)
        normal=np.mean([c.frame[:3]*(1 if int(c.geom1)==cube else -1) for c in near],axis=0);normal=unit(normal)
    else:
        line=np.zeros(6);depth=mujoco.mj_geomDistance(m,d,cube,g,.15,line)
        point=(line[:3]+line[3:])/2;normal=unit(line[3:]-line[:3])
    return float(depth*1000),(point-C)@B*1000,normal

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--margin',type=float,default=1);ap.add_argument('--normal-weight',type=float,default=3);ap.add_argument('--y',type=float,default=10);ap.add_argument('--tag',required=True);ap.add_argument('--max-nfev',type=int,default=180);ap.add_argument('--method',choices=('least_squares','cobyqa'),default='least_squares');a=ap.parse_args()
    out=O/'actual_contact_endpoint'/a.tag;out.mkdir(parents=True,exist_ok=True)
    m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb'));z=np.load(CP/'end_state.npz');d=restore(m,z);base=d.qpos.copy();pd=d.ctrl-base[:16];cube=m.geom('cube').id;C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy();up=z['up'];forward=unit(B0[:,2]-up*(B0[:,2]@up));side=unit(np.cross(up,forward));basis=np.column_stack([up,forward,side]);y=-up;B=np.column_stack([unit(np.cross(y,forward)),y,forward]);quat=Rotation.from_matrix(B).as_quat();hand,_,check=inspect_for(m);geoms=precompute(m,hand);lo,hi,nlo,nhi=native(m);ids=np.array(SIM_TO_MOTOR_IDS)-1;low=np.maximum((lo[ids]+4-2048)*TICK-pd,nlo);high=np.minimum((hi[ids]-4-2048)*TICK-pd,nhi)
    normals=[]
    for name in ('mf_tip','th_tip'):
        g=m.geom(name).id;c=min((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,g}),key=lambda c:c.dist);n=c.frame[:3]*(1 if int(c.geom1)==cube else -1);G=d.geom_xmat[g].reshape(3,3);normals.append(G.T@n)
    ep=np.load(O/'supported_endpoint/lift5_root64/endpoint.npz');seed=ticks_to_sim(ep['goal'])-pd;shift=basis.T@(ep['cube_center']-C0)*1000
    lowx=np.r_[low[A],0,-30,-30,0,-23];highx=np.r_[high[A],40,30,30,22,23]
    x0=np.r_[np.clip(seed[A],low[A]+1e-5,high[A]-1e-5),np.clip(shift,[.001,-29.9,-29.9],[39.9,29.9,29.9]),a.y,12.]
    counter=0
    def measure(x,exact=False):
        C=C0+basis@x[K:K+3]/1000;d.qpos[:]=base;d.qpos[A]=x[:K];d.qpos[-7:]=np.r_[C,quat[3],quat[:3]];d.ctrl[:]=d.qpos[:16]+pd;d.qvel[:]=0;mujoco.mj_forward(m,d)
        contacts=[];norm=[]
        for i,name in enumerate(('mf_tip','th_tip')):
            g=m.geom(name).id;dist,p,n=contact_geometry(m,d,cube,g,C,B);contacts.append((dist,p,n))
            sign=1 if i==0 else -1;norm.extend((d.geom_xmat[g].reshape(3,3)@normals[i]-B[:,0]*sign)*a.normal_weight)
        support=contact_geometry(m,d,cube,m.geom('if_bs_collision_1').id,C,B);cl=gaps(m,d,C,B,geoms,exact);su,sp=check(d);pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
        return contacts,np.array(norm),support,cl,max(sp.values(),default=0.),max(su.values(),default=0.),pen,C
    def residual(x):
        nonlocal counter
        contacts,n,support,cl,selfmm,supportmm,pen,C=measure(x);e=[]
        for i,(dist,p,normal) in enumerate(contacts):
            e.extend([(dist+.15)*2,(p[1]-x[K+3])*.1,(p[2]-x[K+4])*3]);sign=1 if i==0 else -1;e.extend((normal-B[:,0]*sign)*3)
        sd,sp,sn=support
        counter+=1
        return np.r_[e,n,(sd+.15)*2,max(0,25-sp[1])*2,np.maximum(a.margin-np.array([v for name,v in cl]),0)*8,(x[:K]-base[A])*.05,x[K:K+3]*.015,selfmm*500,supportmm*500,max(0,pen-.8)*100]
    began=time.perf_counter()
    if a.method=='least_squares':fit=least_squares(residual,x0,bounds=(lowx,highx),diff_step=1e-5,max_nfev=a.max_nfev,ftol=1e-8,xtol=1e-8,gtol=1e-8)
    else:
        scale=np.r_[np.full(K,.08),np.full(3,3.),np.full(2,4.)]
        opt=minimize(lambda u:float(np.sum(residual(x0+u*scale)**2)),np.zeros(len(x0)),bounds=list(zip((lowx-x0)/scale,(highx-x0)/scale)),method='COBYQA',options={'maxfev':a.max_nfev,'initial_tr_radius':1.,'final_tr_radius':.001,'feasibility_tol':1e-8})
        from types import SimpleNamespace
        fit=SimpleNamespace(x=x0+opt.x*scale)
    contacts,n,support,cl,selfmm,supportmm,pen,C=measure(fit.x,True);gap=min(v for name,v in cl);alignment=float(abs(contacts[0][1][2]-contacts[1][1][2]));accepted=bool(gap>.3 and selfmm<=1e-6 and supportmm<=.15 and pen<=1 and alignment<1.5 and max(abs(c[0]) for c in contacts)<.8 and abs(support[0])<.8)
    report=dict(status='actual_CAD_contact_balanced_endpoint_only',parameters=vars(a),function_evaluations=counter,contact_positions_mm=[c[1].tolist() for c in contacts],contact_distances_mm=[c[0] for c in contacts],opposed_contact_tangent_difference_mm=alignment,index_support_distance_mm=support[0],index_support_point_mm=support[1].tolist(),minimum_full_sweep_gap_mm=gap,self_mm=selfmm,support_mm=supportmm,cube_mm=pen,cube_center_offset_mm=fit.x[K:K+3].tolist(),goal=motor(d.ctrl).tolist(),endpoint_only_accepted=accepted,elapsed_seconds=time.perf_counter()-began,closest_geoms=sorted(cl,key=lambda r:r[1])[:6],cube_pinned_for_proposal_only=True,dynamic_or_physical_success_verified=False,hardware_execution_allowed=False,devices_opened=False)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    if accepted:np.savez_compressed(out/'endpoint.npz',goal=motor(d.ctrl),cube_center=C,cube_rotation=B)
    print(json.dumps(report),flush=True)
if __name__=='__main__':main()
