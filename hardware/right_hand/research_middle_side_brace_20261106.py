"""Propose a third lower-side contact before the index withdraws its support.

The middle finger braces a side face, unlike earlier underside-reach trials.
Pinned geometry is proposal-only; every accepted goal is freely replayed from
the89deg IF/TH grasp with all original dynamic guards and the .15Nm model.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.stats import qmc
from dynamic_common_grip_search_20261105 import ROOT,R,ramp
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from hand_motor_mapping import TICK
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from canonical_grip_dynamics_20261105 import run

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--finger',choices=('middle','ring'),default='middle');args=ap.parse_args();prefix='mf_' if args.finger=='middle' else 'rf_';role=prefix+'tip'
    source=ROOT/'countergrip_level/seed0_source1_lift2_normal1.5_n850/verified_end_state.npz';model=ROOT/'torque_cap_sensitivity/cap0.15/model.mjb';z=np.load(source);m=mujoco.MjModel.from_binary_path(str(model));d=restore(m,z);base=d.qpos.copy();cube=m.geom('cube').id;tip=m.geom(role).id;C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();active=np.arange(4,8) if args.finger=='middle' else np.arange(8,12);lo,hi,nlo,nhi=native(m);lower=nlo[active]+8*TICK;upper=nhi[active]-8*TICK;hand,_,check=inspect_for(m);geom=precompute(m,hand);vertices={g:v for g,v,e in geom};mf_geom=[v for v in geom if m.body(int(m.geom_bodyid[v[0]])).name.startswith(prefix)];out=ROOT/(args.finger+'_side_brace');out.mkdir(exist_ok=True);results=[];began=time.perf_counter()
    for sign in (1,-1):
        normal=B[:,0]*sign
        def measure(x,full=False):
            d.qpos[:]=base;d.qpos[active]=x;d.qvel[:]=0;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d);world=vertices[tip]@d.geom_xmat[tip].reshape(3,3).T+d.geom_xpos[tip];projection=(world-C)@normal;weights=np.exp(-(projection-min(projection))/.0002);point=np.average(world,axis=0,weights=weights);local=(point-C)@B*1000;gap=(min(projection)-.0275)*1000
            if not full:return gap,local
            jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,point,int(m.geom_bodyid[tip]));bias=d.qfrc_bias[active]-d.qfrc_passive[active];tau=bias-normal@jp[:,active]*.6;control=x+tau/m.actuator_gainprm[active,0];su,sp=check(d);clear=min(v for name,v in gaps(m,d,C,B,mf_geom,True));pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            return gap,local,point,jp,bias,tau,control,clear,max(sp.values(),default=0.),max(su.values(),default=0.),pen
        samples=qmc.scale(qmc.Sobol(4,scramble=True,seed=7211+sign).random_base2(11),lower,upper);samples=np.vstack([np.clip(base[active],lower,upper),samples]);scores=[]
        for x in samples:
            gap,p=measure(x);scores.append(gap**2*10+(p[1]-14)**2+max(0.,abs(p[2])-20)**2*10+np.sum((x-base[active])**2)*.05)
        seeds=[]
        for index in np.argsort(scores):
            if all(np.linalg.norm(samples[index]-s)>.3 for s in seeds):seeds.append(samples[index])
            if len(seeds)>=4:break
        for index,seed in enumerate(seeds):
            folder=out/f'x{sign:+d}_seed{index}';folder.mkdir(exist_ok=True)
            def residual(x):
                gap,p,point,jp,bias,tau,ctrl,clear,selfmm,supportmm,pen=measure(x,True)
                return np.r_[(gap+.03)*10,(p[1]-14)*2,p[2]*.2,max(0.,abs(p[2])-21)*20,np.maximum(nlo[active]+4*TICK-ctrl,0)*500,np.maximum(ctrl-nhi[active]+4*TICK,0)*500,np.maximum(abs(tau)-.15,0)*1000,max(0.,.8-clear)*20,(x-base[active])*.1,selfmm*500000,supportmm*1000,max(0.,pen-.8)*1000]
            fit=least_squares(residual,np.clip(seed,lower+1e-9,upper-1e-9),bounds=(lower,upper),max_nfev=180,ftol=1e-9,xtol=1e-9,gtol=1e-9);gap,p,point,jp,bias,tau,ctrl,clear,selfmm,supportmm,pen=measure(fit.x,True);accepted=bool(abs(gap)<.2 and abs(p[2])<22 and 10<p[1]<23 and clear>.3 and selfmm<=1e-6 and supportmm<=.15 and pen<=1 and np.all(ctrl>=nlo[active]) and np.all(ctrl<=nhi[active]) and max(abs(tau))<=.15);np.savez_compressed(folder/'static_proposal.npz',actual_joint_proposal=d.qpos[:16],cube_center=C,cube_rotation=B);row=dict(mode=f'x{sign:+d}',seed=index,static_accepted=accepted,gap_mm=gap,point_cube_local_mm=p.tolist(),middle_only_upper_gap_mm=clear,self_mm=selfmm,support_mm=supportmm,cube_mm=pen,dynamic=[])
            if accepted:
                for force in (.3,.6,1.):
                    control=z['ctrl'].copy();control[active]=fit.x+(bias-normal@jp[:,active]*force)/m.actuator_gainprm[active,0];goal=motor(control)
                    if np.any(goal<lo) or np.any(goal>hi):row['dynamic'].append(dict(force_N=force,native_goal_rejected=True));continue
                    rows=ramp(motor(z['ctrl']),goal,134);np.savez_compressed(folder/f'force{force:g}.npz',start=motor(z['ctrl']),goals=rows);v=run(m,z,rows,1,folder/f'force{force:g}_end.npz',None,True,5,('if_tip',role,'th_tip'));row['dynamic'].append(dict(force_N=force,result=v))
            results.append(row);report=dict(status='third_finger_side_brace_proposals_and_free_replays',finger=args.finger,source_state=source.relative_to(R).as_posix(),source_state_sha256=sha(source),source_model_sha256=sha(model),cases=results,static_cube_pinning_not_motion_evidence=True,third_finger_clearance_is_not_whole_hand_clearance=True,devices_opened=False,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(mode=row['mode'],seed=index,static=accepted,gap=gap,height=p[1],side=p[2],clearance=clear,dynamic=[dict(force=t['force_N'],safety=t.get('result',{}).get('safety_passed'),yaw=t.get('result',{}).get('final_yaw_from_placement_deg'),contact_fraction=t.get('result',{}).get('settled_tail',{}).get('opposed_lower_body_contact_fraction')) for t in row['dynamic']])),flush=True)
if __name__=='__main__':main()
