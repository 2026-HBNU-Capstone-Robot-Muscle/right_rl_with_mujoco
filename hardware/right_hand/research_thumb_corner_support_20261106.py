"""Seek a force-feasible thumb contact under a cube corner, offline only.

Static pinning proposes poses. Free-cube replay checks the actual approach;
no geometry, native range, torque cap or physical guard is relaxed.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import hashlib,json,time
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.stats import qmc
from dynamic_common_grip_search_20261105 import ROOT,R,ramp
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from canonical_grip_dynamics_20261105 import run

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    raw=json.loads((ROOT/'near_quarter_full_replay/review.json').read_bytes());model=R/raw['source_model'];assert sha(model)==raw['source_model_sha256'];m=mujoco.MjModel.from_binary_path(str(model));source=ROOT/'countergrip_level/seed0_source1_lift2_normal1.5_n850/verified_end_state.npz';z=np.load(source);d=restore(m,z);base=d.qpos.copy();pd=d.ctrl-base[:16];cube=m.geom('cube').id;tip=m.geom('th_tip').id;C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();up=z['up'];normal_target=(B[:,1]+B[:,2])/np.sqrt(2);active=np.arange(12,16);lo,hi,nlo,nhi=native(m);ids=np.asarray(SIM_TO_MOTOR_IDS)-1;lower=nlo[active]+8*TICK;upper=nhi[active]-8*TICK;hand,_,check=inspect_for(m);geom=precompute(m,hand);th_geom=[v for v in geom if m.body(int(m.geom_bodyid[v[0]])).name.startswith('th_')];out=ROOT/'thumb_corner_support';out.mkdir(exist_ok=True);start=motor(z['ctrl']);began=time.perf_counter()
    def measure(x,full=False):
        d.qpos[:]=base;d.qpos[active]=x;d.ctrl[:]=d.qpos[:16]+pd;d.qvel[:]=0;mujoco.mj_forward(m,d);line=np.zeros(6);distance=float(mujoco.mj_geomDistance(m,d,tip,cube,.15,line))*1000;point=line[:3];local=(line[3:]-C)@B*1000;matches=[c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,tip}]
        if matches:
            c=min(matches,key=lambda c:c.dist);normal=c.frame[:3]*(1 if int(c.geom1)==cube else -1);point=c.pos.copy();local=(c.pos-C)@B*1000
        else:
            v=line[:3]-line[3:];normal=v/max(np.linalg.norm(v),1e-12)
        if not full:return distance,local,normal,point
        jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,point,int(m.geom_bodyid[tip]));bias=d.qfrc_bias[active]-d.qfrc_passive[active];tau=bias-(normal_target@jp[:,active])*1.5;control=x+tau/m.actuator_gainprm[active,0];su,sp=check(d);clear=min(v for name,v in gaps(m,d,C,B,th_geom,True));pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
        return distance,local,normal,point,control,tau,jp,bias,clear,max(sp.values(),default=0.),max(su.values(),default=0.),pen,bool(matches)
    samples=qmc.scale(qmc.Sobol(4,scramble=True,seed=311206).random_base2(12),lower,upper);samples=np.vstack([np.clip(base[active],lower,upper),samples]);scores=[]
    for x in samples:
        dist,p,n,point=measure(x);scores.append((dist+.02)**2*10+(p[1]-27.5)**2*4+(p[2]-27.5)**2*4+max(0.,abs(p[0])-20)**2+np.sum((n-normal_target)**2)*100)
    selected=[]
    for index in np.argsort(scores):
        if all(np.linalg.norm(samples[index]-s)>.3 for s in selected):selected.append(samples[index])
        if len(selected)>=6:break
    results=[]
    for index,seed in enumerate(selected):
        folder=out/f'seed{index}';folder.mkdir(exist_ok=True)
        def residual(x):
            dist,p,n,point,control,tau,jp,bias,clear,selfmm,supportmm,pen,contact=measure(x,True)
            return np.r_[(dist+.02)*10,(p[1]-27.5)*4,(p[2]-27.5)*4,max(0.,abs(p[0])-20)*10,p[0]*.1,(n-normal_target)*10,(x-seed)*.1,np.maximum(nlo[active]+4*TICK-control,0)*500,np.maximum(control-nhi[active]+4*TICK,0)*500,np.maximum(abs(tau)-.15,0)*1000,np.maximum(abs(control-x)-.12,0)*500,max(0.,.8-clear)*20,selfmm*500000,supportmm*1000,max(0.,pen-.8)*1000]
        fit=least_squares(residual,np.clip(seed,lower+1e-9,upper-1e-9),bounds=(lower,upper),max_nfev=160,ftol=1e-9,xtol=1e-9,gtol=1e-9);dist,p,n,point,control,tau,jp,bias,clear,selfmm,supportmm,pen,contact=measure(fit.x,True);accepted=bool(contact and abs(dist)<.2 and abs(p[1]-27.5)<.3 and abs(p[2]-27.5)<.3 and abs(p[0])<22 and n@normal_target>.95 and clear>.3 and selfmm<=1e-6 and supportmm<=.15 and pen<=1 and np.all(control>=nlo[active]) and np.all(control<=nhi[active]) and max(abs(tau))<=.15 and max(abs(control-fit.x))<=.12);np.savez_compressed(folder/'static_proposal.npz',actual_joint_proposal=d.qpos[:16],cube_center=C,cube_rotation=B);row=dict(seed=index,static_force_feasible=accepted,distance_mm=dist,point_cube_local_mm=p.tolist(),normal_alignment=float(n@normal_target),thumb_upper_gap_mm=clear,self_mm=selfmm,support_mm=supportmm,cube_mm=pen,force1p5_goal_margin_rad=float(min(min(control-nlo[active]),min(nhi[active]-control))),dynamic=[])
        if accepted:
            actual=d.qpos[:16].copy()
            for force in (.7,1.,1.5):
                ctrl=actual+pd;ctrl[active]=actual[active]+(bias-(normal_target@jp[:,active])*force)/m.actuator_gainprm[active,0];goal=motor(ctrl)
                if np.any(goal<lo) or np.any(goal>hi):row['dynamic'].append(dict(force_N=force,native_goal_rejected=True));continue
                rows=ramp(start,goal,134);np.savez_compressed(folder/f'force{force:g}.npz',start=start,goals=rows);v=run(m,z,rows,1,folder/f'force{force:g}_end.npz',None,True,5,('if_tip','th_tip'));row['dynamic'].append(dict(force_N=force,result=v))
        results.append(row);report=dict(status='force_feasible_thumb_corner_support_proposals_and_free_replays',source_state=source.relative_to(R).as_posix(),source_state_sha256=sha(source),source_model_sha256=sha(model),cases=results,cube_pinned_only_in_proposals=True,no_external_cube_force_or_pose_written_during_replay=True,native_geometry_force_caps_and_dynamic_guards_unchanged=True,devices_opened=False,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(seed=index,static=accepted,point=p.tolist(),normal_alignment=row['normal_alignment'],clearance=clear,dynamic=[dict(force=t['force_N'],safety=t.get('result',{}).get('safety_passed'),yaw=t.get('result',{}).get('final_yaw_from_placement_deg'),tilt=t.get('result',{}).get('final_up_tilt_deg')) for t in row['dynamic']])),flush=True)
if __name__=='__main__':main()
