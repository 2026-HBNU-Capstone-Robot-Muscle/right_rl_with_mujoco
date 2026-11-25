"""Seek the same lower-body contact with more remaining middle-finger travel.

Cube pose is pinned only during IK. Every accepted proposal requires a separate
continuous free-cube replay; this file alone never proves a realizable grip.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,json,time
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from dynamic_common_grip_search_20261105 import ROOT
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from hand_motor_mapping import TICK,SIM_TO_MOTOR_IDS
from upper_layer_cylinder_clearance_20261105 import precompute,gaps

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--joint-weight',type=float,choices=(1.,3.,10.),required=True);ap.add_argument('--shift-mm',type=float,choices=(0.,10.,20.),default=0.);a=ap.parse_args()
    out=ROOT/f'workspace_grip/weight{a.joint_weight:g}_shift{a.shift_mm:g}';out.mkdir(parents=True,exist_ok=True);folder=ROOT/'full_placement_v3_thumb12/front-10/g1_f0.3';m=mujoco.MjModel.from_binary_path(str(folder/'model.mjb'));z=np.load(folder/'end_state.npz');d=restore(m,z);base=d.qpos.copy();pd=d.ctrl-base[:16];cube=m.geom('cube').id;C0=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();q=Rotation.from_matrix(B).as_quat();lo,hi,nlo,nhi=native(m);ids=np.array(SIM_TO_MOTOR_IDS)-1;low=np.maximum((np.ceil(lo[ids])+4-2048)*TICK-pd,nlo);high=np.minimum((np.floor(hi[ids])-4-2048)*TICK-pd,nhi);A=np.r_[0,np.arange(4,8),np.arange(12,16)];K=len(A);hand,_,check=inspect_for(m);geometry=precompute(m,hand);anchors=[]
    for name in ('mf_tip','th_tip','if_bs_collision_1'):
        gid=m.geom(name).id;cs=[c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,gid}]
        if not cs:
            if name=='if_bs_collision_1':continue
            raise ValueError('source lower contact missing')
        c=min(cs,key=lambda c:c.dist);G=d.geom_xmat[gid].reshape(3,3);normal=c.frame[:3]*(1 if int(c.geom1)==cube else -1);anchors.append((gid,G.T@(c.pos-d.geom_xpos[gid]),G.T@normal,B.T@(c.pos-C0),B.T@normal,name))
    # Reserve travel for the next reorientation. These are task preferences,
    # never enlarged joint bounds or physical-limit changes.
    desired=motor(d.ctrl);desired[9]=2350;desired[10]=2700;desired[8]=2950;desired[11]=2600;desired_sim=(desired[ids]-2048)*TICK-pd
    seed=np.r_[base[A],z['up']*a.shift_mm];lower=np.r_[low[A],[-35.]*3];upper=np.r_[high[A],[35.]*3]
    if np.any(seed<lower) or np.any(seed>upper):raise ValueError('invalid optimizer seed')
    def measure(x,exact=False):
        C=C0+x[-3:]/1000;d.qpos[:]=base;d.qpos[A]=x[:K];d.qpos[-7:]=np.r_[C,q[3],q[:3]];d.ctrl[:]=d.qpos[:16]+pd;d.qvel[:]=0;mujoco.mj_forward(m,d);errors=[];normals=[]
        for gid,local,ln,cl,cn,name in anchors:
            G=d.geom_xmat[gid].reshape(3,3);errors.extend((d.geom_xpos[gid]+G@local-(C+B@cl))*1000*(.5 if name=='if_bs_collision_1' else 1.))
            if name!='if_bs_collision_1':normals.extend((G@ln-B@cn)*3.)
        clear=gaps(m,d,C,B,geometry,exact);su,sp=check(d);pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
        return np.array(errors),np.array(normals),clear,max(sp.values(),default=0.),max(su.values(),default=0.),pen
    def residual(x):
        e,n,clear,selfmm,supportmm,pen=measure(x);joint=(x[:K]-desired_sim[A])*a.joint_weight;return np.r_[e,n,joint,np.maximum(.8-np.array([v for name,v in clear]),0)*10,x[-3:]*.01,selfmm*500000,supportmm*1000,max(0.,pen-.8)*100]
    began=time.perf_counter();fit=least_squares(residual,seed,bounds=(lower,upper),max_nfev=180,ftol=1e-8,xtol=1e-8,gtol=1e-8);e,n,clear,selfmm,supportmm,pen=measure(fit.x,True);goal=motor(d.ctrl);error=float(max(np.linalg.norm(e.reshape(len(anchors),3),axis=1)));gap=min(v for name,v in clear);accepted=bool(error<.8 and gap>.3 and selfmm<=1e-6 and supportmm<=.15 and pen<=1 and np.all(goal>=lo) and np.all(goal<=hi))
    report=dict(status='lower_contact_workspace_margin_IK_proposal_only',joint_weight=a.joint_weight,initial_shift_mm=a.shift_mm,anchor_error_mm=error,minimum_upper_gap_mm=gap,self_mm=selfmm,support_mm=supportmm,cube_mm=pen,translation_world_mm=fit.x[-3:].tolist(),goal=goal.tolist(),middle_order_ids=[12,10,11,9],middle_goal_ticks=goal[np.array([12,10,11,9])-1].tolist(),middle_preferred_ticks=desired[np.array([12,10,11,9])-1].tolist(),proposal_accepted=accepted,elapsed_seconds=time.perf_counter()-began,pinned_cube_only_for_proposal=True,free_cube_dynamics_not_replayed=True,all_original_limits_and_collision_meshes_unchanged=True,hardware_execution_allowed=False,devices_opened=False)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    if accepted:np.savez_compressed(out/'endpoint.npz',goal=goal,cube_center=C0+fit.x[-3:]/1000,cube_rotation=B)
    print(json.dumps(report),flush=True)
if __name__=='__main__':main()
