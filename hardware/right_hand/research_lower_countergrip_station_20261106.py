"""Seek a nearby upright90 lower IF/TH grip, proposals only.

Translation of the pinned cube occurs only during static optimization, never
as evidence of a motion. Actual motion must independently replay this proposal.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import restore,inspect_for,native
from hand_motor_mapping import TICK
from cube_task_frame_20261105 import upright_yaw_target
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from research_quasistatic_wrench_path_20261105 import allocate_native
from research_rotation_body_hold_20261104 import motor

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--force-N',type=float,choices=(1.5,2.,2.5),default=1.5);ap.add_argument('--span-mm',type=float,choices=(8.,12.),default=8.);ap.add_argument('--force-equilibrium',action='store_true');a=ap.parse_args()
    source=ROOT/'countergrip_level/seed0_source1_lift0_normal2.5_n900_resume_lower_staged_freebase/end_state.npz';model=ROOT/'torque_cap_sensitivity/cap0.15/model.mjb';z=np.load(source);m=mujoco.MjModel.from_binary_path(str(model));d=restore(m,z);base=d.qpos.copy();C0=d.qpos[-7:-4].copy();B=upright_yaw_target(z['initial_cube_rotation'],z['up'],90);quat=Rotation.from_matrix(B).as_quat();quat=np.r_[quat[3],quat[:3]];cube=m.geom('cube').id;active=np.r_[np.arange(4),np.arange(12,16)];lo,hi,nlo,nhi=native(m);hand,_,check=inspect_for(m);geometry=precompute(m,hand);vertices={g:v for g,v,e in geometry};tag='lower_countergrip_station'+(f'_force{a.force_N:g}_span{a.span_mm:g}' if a.force_N!=1.5 or a.span_mm!=8 else '')+('_equilibrium' if a.force_equilibrium else '');out=ROOT/tag;out.mkdir(exist_ok=True);start=time.perf_counter();lower=np.r_[nlo[active]+8*TICK,[-a.span_mm]*3];upper=np.r_[nhi[active]-8*TICK,[a.span_mm]*3];results=[]
    normals={'if_tip':-B[:,2],'th_tip':B[:,2]}
    def measure(x,exact=False):
        C=C0+x[-3:]/1000;d.qpos[:]=base;d.qpos[active]=x[:-3];d.qpos[-7:-4]=C;d.qpos[-4:]=quat;d.qvel[:]=0;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d);tasks=[];details=[];controls=[];taus=[]
        for name,normal in normals.items():
            g=m.geom(name).id;world=vertices[g]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g];proj=(world-C)@normal;weights=np.exp(-(proj-min(proj))/.0002);point=np.average(world,axis=0,weights=weights);local=(point-C)@B*1000;gap=(min(proj)-.0275)*1000;jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,point,int(m.geom_bodyid[g]));joints=np.arange(4) if name=='if_tip' else np.arange(12,16);tau=d.qfrc_bias[joints]-d.qfrc_passive[joints]-(normal@jp[:,joints])*a.force_N;ctrl=d.qpos[joints]+tau/m.actuator_gainprm[joints,0];controls.extend(ctrl);taus.extend(tau);tasks.extend([(gap+.03)*10,local[0]*.3,(local[1]-14)*2]);details.append(dict(name=name,gap_mm=gap,point_local_mm=local.tolist()))
        clearance=gaps(m,d,C,B,geometry,exact);su,sp=check(d);pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
        return np.asarray(tasks),details,clearance,max(sp.values(),default=0.),max(su.values(),default=0.),pen,np.asarray(controls),np.asarray(taus)
    for index,offset in enumerate(([0,0,0],[-4,0,0],[0,-4,0],[0,0,4],[4,4,0],[0,4,-4])):
        seed=np.r_[np.clip(base[active],lower[:-3]+1e-8,upper[:-3]-1e-8),offset]
        def residual(x):
            task,details,clear,selfmm,supportmm,pen,ctrl,tau=measure(x)
            return np.r_[task,np.maximum(.8-np.array([v for name,v in clear]),0)*10,np.maximum(nlo[active]+4*TICK-ctrl,0)*500,np.maximum(ctrl-nhi[active]+4*TICK,0)*500,np.maximum(abs(tau)-.15,0)*1000,(x[:-3]-base[active])*.1,x[-3:]*.12,selfmm*500000,supportmm*1000,max(0.,pen-.8)*1000]
        fit=least_squares(residual,seed,bounds=(lower,upper),max_nfev=180,ftol=1e-9,xtol=1e-9,gtol=1e-9);task,details,clear,selfmm,supportmm,pen,ctrl,tau=measure(fit.x,True);accepted_geometry=bool(all(abs(t['gap_mm'])<.2 and abs(t['point_local_mm'][0])<22 and 10<t['point_local_mm'][1]<23 for t in details) and min(v for name,v in clear)>.3 and selfmm<=1e-6 and supportmm<=.15 and pen<=1 and np.all(ctrl>=nlo[active]) and np.all(ctrl<=nhi[active]) and max(abs(tau))<=.15);folder=out/f'seed{index}';folder.mkdir(exist_ok=True);force_ctrl=None;allocation=None;equilibrium=False
        if a.force_equilibrium and accepted_geometry:
            F=-m.body_mass[m.body('cube').id]*m.opt.gravity;force_ctrl,allocation=allocate_native(m,d,cube,d.qpos[-7:-4],F,np.zeros(3),{'if_tip':a.force_N,'th_tip':a.force_N},hand,True,(nlo,nhi),('if_tip','th_tip'),normal_reference_weight=2.)
            if force_ctrl is not None:
                error=np.asarray(allocation['desired_wrench_tracking_residual']);native_goals=motor(force_ctrl);equilibrium=bool(max(abs(error[:3]))<.005 and max(abs(error[3:]))<.0002 and np.all(native_goals>=lo) and np.all(native_goals<=hi) and all(allocation['normal_N'].get(n,0)>.2 for n in ('if_tip','th_tip')))
        accepted=accepted_geometry and (not a.force_equilibrium or equilibrium);np.savez_compressed(folder/'static_proposal.npz',actual_joint_proposal=d.qpos[:16],cube_center=d.qpos[-7:-4],cube_rotation=B,optimizer_vector=fit.x,**({'force_balanced_control':force_ctrl} if equilibrium else {}));row=dict(seed=index,accepted_static_only=accepted,accepted_geometry_only=accepted_geometry,full_static_gravity_equilibrium_passed=equilibrium,allocation=None if allocation is None else {k:v for k,v in allocation.items() if k!='contact_tasks'},translation_mm=fit.x[-3:].tolist(),contacts=details,upper_gap_mm=min(v for name,v in clear),self_mm=selfmm,support_mm=supportmm,cube_mm=pen,torque_max_nm=float(max(abs(tau))),goal_margin_rad=float(min(min(ctrl-nlo[active]),min(nhi[active]-ctrl))),cost=float(np.dot(task,task)));results.append(row);(out/'review.json').write_text(json.dumps(dict(status='nearby_upright90_lower_grip_static_proposals',parameters=vars(a),source_state=source.relative_to(R).as_posix(),source_state_sha256=sha(source),source_model_sha256=sha(model),cases=results,static_cube_pinning_not_motion_evidence=True,devices_opened=False,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-start),indent=2)+'\n',encoding='utf-8');print(json.dumps(row),flush=True)
if __name__=='__main__':main()
