"""Seek a lower MF/TH grasp with independent index-root support.

An upright pose that balances gravity at two point contacts can still tip.
This proposal keeps a third underside contact, tests the complete native
wrench, then independently releases and holds the cube. It is not yet a
trajectory from the original placement and never authorizes real motion.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from hand_motor_mapping import TICK,ticks_to_sim
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from research_quasistatic_wrench_path_20261105 import allocate_native
from canonical_grip_dynamics_20261105 import run
from checkpoint_quarter_review_20261106 import quarter_reasons

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--contact-depth-mm',type=float,choices=(10.,14.),default=14.);ap.add_argument('--support-weight',type=float,choices=(10.,30.),default=10.);a=ap.parse_args()
    model=ROOT/'torque_cap_sensitivity/cap0.15/model.mjb';reference=ROOT/'near_quarter_full_replay/end_state.npz';seedpath=ROOT/'postturn_regrasp_refined/middle_zneg/static_proposal.npz';z=dict(np.load(reference));seedp=np.load(seedpath);m=mujoco.MjModel.from_binary_path(str(model));d=restore(m,z);base=d.qpos.copy();base[:16]=seedp['actual_joint_proposal'];C0=seedp['cube_center'];B=upright_yaw_target(z['initial_cube_rotation'],z['up'],90);q=Rotation.from_matrix(B).as_quat();quat=np.r_[q[3],q[:3]];active=np.r_[np.arange(8),np.arange(12,16)];cube=m.geom('cube').id;support=m.geom('if_bs_collision_1').id;hand,_,check=inspect_for(m);geometry=precompute(m,hand);vertices={g:v for g,v,e in geometry};lo,hi,nlo,nhi=native(m);lower=np.r_[nlo[active]+8*TICK,[-12]*3];upper=np.r_[nhi[active]-8*TICK,[12]*3];tag='three_contact_handoff'+(f'_depth{a.contact_depth_mm:g}_support{a.support_weight:g}' if a.contact_depth_mm!=14 or a.support_weight!=10 else '');out=ROOT/tag;out.mkdir(exist_ok=True);cases=[];began=time.perf_counter()
    def measure(x,exact=False):
        C=C0+x[-3:]/1000;d.qpos[:]=base;d.qpos[active]=x[:-3];d.qpos[-7:-4]=C;d.qpos[-4:]=quat;d.qvel[:]=0;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d);tasks=[];points=[];ctrls=[];taus=[]
        for name,normal in (('mf_tip',-B[:,2]),('th_tip',B[:,2]),('if_bs_collision_1',B[:,1])):
            g=m.geom(name).id;world=vertices[g]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g];project=(world-C)@normal;weights=np.exp(-(project-min(project))/.0002);point=np.average(world,axis=0,weights=weights);local=(point-C)@B*1000;gap=(min(project)-.0275)*1000
            if g==support:tasks.extend([(gap+.02)*a.support_weight,max(0.,abs(local[0])-23)*10,max(0.,abs(local[2])-23)*10]);joints=np.array([0]);force=.98
            else:tasks.extend([(gap+.03)*10,(local[1]-a.contact_depth_mm)*2,local[0]*.3,max(0.,abs(local[0])-22)*10]);joints=np.arange(4,8) if name=='mf_tip' else np.arange(12,16);force=1.5
            jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,point,int(m.geom_bodyid[g]));tau=d.qfrc_bias[joints]-d.qfrc_passive[joints]-(normal@jp[:,joints])*force;ctrl=d.qpos[joints]+tau/m.actuator_gainprm[joints,0];ctrls.extend(ctrl);taus.extend(tau);points.append(dict(name=name,gap_mm=gap,local_mm=local.tolist(),point=point.tolist()))
        clear=gaps(m,d,C,B,geometry,exact);su,sp=check(d);pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.);force_joints=np.r_[np.arange(4,8),np.arange(12,16),[0]]
        return np.asarray(tasks),points,clear,max(sp.values(),default=0.),max(su.values(),default=0.),pen,np.asarray(ctrls),np.asarray(taus),force_joints
    offsets=([0,0,0],[-4,0,0],[0,0,-4],[0,-4,-4],[4,0,-4],[0,4,-4])
    for index,offset in enumerate(offsets):
        seed=np.r_[np.clip(base[active],lower[:-3]+1e-8,upper[:-3]-1e-8),offset]
        if index>=2:seed[0]=np.clip(seed[0]-.1,lower[0]+1e-8,upper[0]-1e-8)
        def residual(x):
            task,points,clear,selfmm,supportmm,pen,ctrl,tau,j=measure(x)
            return np.r_[task,np.maximum(1.2-np.array([v for name,v in clear]),0)*10,np.maximum(nlo[j]+4*TICK-ctrl,0)*500,np.maximum(ctrl-nhi[j]+4*TICK,0)*500,np.maximum(abs(tau)-.15,0)*1000,(x[:-3]-base[active])*.1,x[-3:]*.1,selfmm*500000,supportmm*1000,max(0.,pen-.8)*1000]
        fit=least_squares(residual,seed,bounds=(lower,upper),max_nfev=220,ftol=1e-9,xtol=1e-9,gtol=1e-9);task,points,clear,selfmm,supportmm,pen,ctrl,tau,j=measure(fit.x,True);inside=all(abs(p['gap_mm'])<.2 and all(abs(p['local_mm'][k])<26 for k in ((0,2) if p['name']=='if_bs_collision_1' else (0,1))) for p in points);geometry_ok=bool(inside and min(v for name,v in clear)>.3 and selfmm<=1e-6 and supportmm<=.15 and pen<=1);allocation=None;balanced=None;feasible=False;verification=None;why=['static_geometry_or_force_not_proved'];folder=out/f'seed{index}';folder.mkdir(exist_ok=True)
        if geometry_ok:
            balanced,allocation=allocate_native(m,d,cube,d.qpos[-7:-4],-m.body_mass[m.body('cube').id]*m.opt.gravity,np.zeros(3),{'mf_tip':1.5,'th_tip':1.5,'if_bs_collision_1':1.},hand,True,(nlo,nhi),('mf_tip','th_tip'),normal_reference_weight=2.)
            if balanced is not None:
                r=np.asarray(allocation['desired_wrench_tracking_residual']);goals=motor(balanced);feasible=bool(max(abs(r[:3]))<.005 and max(abs(r[3:]))<.0002 and np.all(goals>=lo) and np.all(goals<=hi))
                if feasible:
                    initial=dict(z);initial['qpos']=d.qpos.copy();initial['qvel']=np.zeros_like(d.qvel);initial['qacc_warmstart']=np.zeros_like(d.qacc_warmstart);initial['ctrl']=ticks_to_sim(goals);initial['required_lower_contacts']=np.asarray(['mf_tip','th_tip']);np.savez_compressed(folder/'independent_initial_state.npz',**initial);rows=np.tile(goals,(134,1));verification=run(m,initial,rows,1,folder/'hold_end.npz',folder/'trajectory.npz',True,5,('mf_tip','th_tip'));end=restore(m,np.load(folder/'hold_end.npz'));why=quarter_reasons(verification,initial['qpos'][-7:-4],True,orientation_error_deg(end.xmat[m.body('cube').id].reshape(3,3),B))
        np.savez_compressed(folder/'static_proposal.npz',actual_joint_proposal=fit.x[:-3],cube_center=C0+fit.x[-3:]/1000,cube_rotation=B,active_sim_indices=active,**({'force_balanced_control':balanced} if feasible else {}));row=dict(seed=index,geometry_passed=geometry_ok,static_native_gravity_feasible=feasible,contacts=points,upper_gap_mm=min(v for name,v in clear),self_mm=selfmm,support_mm=supportmm,cube_mm=pen,translation_mm=fit.x[-3:].tolist(),allocation=None if allocation is None else {k:v for k,v in allocation.items() if k!='contact_tasks'},independent_hold=verification,independent_hold90_passed=not why,reasons=why);cases.append(row);report=dict(status='three_contact_static_handoff_and_independent_hold',source_model_sha256=sha(model),source_reference_state_sha256=sha(reference),source_seed_proposal_sha256=sha(seedpath),cases=cases,not_a_reaching_or_reorientation_trajectory=True,devices_opened=False,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(seed=index,geometry=geometry_ok,force=feasible,hold90=not why,gap=row['upper_gap_mm'],contacts=[{k:v for k,v in p.items() if k!='point'} for p in points],reasons=why)),flush=True)
if __name__=='__main__':main()
