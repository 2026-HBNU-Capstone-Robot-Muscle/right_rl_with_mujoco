"""Bounded static IK alternatives for thumb/front and index/back lower grip.

The middle finger may share the back face or support the bottom. Any generated
pose is only a proposal until separately held with the cube free. No devices.
"""
from pathlib import Path
import argparse,hashlib,json,sys,time
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from scipy.optimize import least_squares
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from research_quasistatic_wrench_path_20261105 import allocate_native
from canonical_grip_dynamics_20261105 import run
from hand_motor_mapping import TICK,ticks_to_sim


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cube-span-mm',type=int,choices=(6,12),default=6)
    parser.add_argument('--back-x-mm',type=int,choices=(10,20),default=10)
    parser.add_argument('--back-only',action='store_true')
    args=parser.parse_args()
    tag='three_finger_trials' if (args.cube_span_mm,args.back_x_mm,args.back_only)==(6,10,False) else f'three_finger_trials_span{args.cube_span_mm}_back{args.back_x_mm}'+('_back_only' if args.back_only else '')
    out=ROOT/'right_hand_rotation_20261116/opposed_lower_grip'/tag
    out.mkdir(exist_ok=False)
    model=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
    source=out.parent/'independent_initial.npz';z=dict(np.load(source,allow_pickle=False))
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':model.read_bytes()})
    d=restore(m,z);base=d.qpos.copy();center=base[-7:-4].copy();B=d.geom_xmat[m.geom('cube').id].reshape(3,3).copy()
    active=np.r_[np.arange(8),np.arange(12,16)];lo,hi,nlo,nhi=native(m)
    hand,_,check=inspect_for(m);geometry=precompute(m,hand);vertices={g:v for g,v,e in geometry}
    lower=np.r_[nlo[active]+8*TICK,[-args.cube_span_mm]*3];upper=np.r_[nhi[active]-8*TICK,[args.cube_span_mm]*3]
    cube=m.geom('cube').id;began=time.perf_counter();cases=[]
    for variant in (('middle_back',) if args.back_only else ('middle_back','middle_bottom')):
        for seed_index,offset in enumerate(([0,0,0],[0,0,-3])):
            name=f'{variant}_{seed_index}';folder=out/name;folder.mkdir()
            specs=[('th_tip',B[:,2],12,2.,0.,14.,(0,1)),
                   ('if_tip',-B[:,2],0,1.25,-float(args.back_x_mm) if variant=='middle_back' else 0.,14.,(0,1))]
            if variant=='middle_back':specs.append(('mf_tip',-B[:,2],4,1.25,float(args.back_x_mm),14.,(0,1)))
            else:specs.append(('mf_tip',B[:,1],4,1.,0.,-10.,(0,2)))
            def measure(x,exact=False):
                C=center+x[-3:]/1000;d.qpos[:]=base;d.qpos[active]=x[:-3]
                d.qpos[-7:-4]=C;d.qvel[:]=0;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
                task=[];contacts=[];ctrls=[];taus=[];joints=[]
                for geom,normal,start,force,t0,t1,axes in specs:
                    g=m.geom(geom).id;world=vertices[g]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
                    projected=(world-C)@normal;weights=np.exp(-(projected-projected.min())/.0002)
                    point=np.average(world,axis=0,weights=weights);local=(point-C)@B*1000
                    gap=(projected.min()-.0275)*1000
                    task.extend([(gap+.03)*10,(local[axes[0]]-t0),2*(local[axes[1]]-t1)])
                    j=np.arange(start,start+4);jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,point,int(m.geom_bodyid[g]))
                    tau=d.qfrc_bias[j]-d.qfrc_passive[j]-(normal@jp[:,j])*force
                    control=d.qpos[j]+tau/m.actuator_gainprm[j,0]
                    ctrls.extend(control);taus.extend(tau);joints.extend(j)
                    contacts.append(dict(geom=geom,gap_mm=float(gap),cube_local_mm=local.tolist()))
                clear=gaps(m,d,C,B,geometry,exact);su,sp=check(d)
                pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
                return np.array(task),contacts,clear,max(sp.values(),default=0.),max(su.values(),default=0.),pen,np.array(ctrls),np.array(taus),np.array(joints)
            def residual(x):
                task,_,clear,selfmm,supportmm,pen,ctrl,tau,j=measure(x)
                return np.r_[task,np.maximum(.8-np.array([v for _,v in clear]),0)*10,
                    np.maximum(nlo[j]+4*TICK-ctrl,0)*500,np.maximum(ctrl-nhi[j]+4*TICK,0)*500,
                    np.maximum(abs(tau)-.15,0)*1000,(x[:-3]-base[active])*.1,x[-3:]*.12,
                    selfmm*500000,supportmm*1000,max(0.,pen-.8)*1000]
            seed=np.r_[np.clip(base[active],lower[:-3]+1e-8,upper[:-3]-1e-8),offset]
            fit=least_squares(residual,seed,bounds=(lower,upper),max_nfev=160,ftol=1e-8,xtol=1e-8,gtol=1e-8)
            task,contacts,clear,selfmm,supportmm,pen,ctrl,tau,j=measure(fit.x,True)
            geometry_ok=bool(all(abs(c['gap_mm'])<.2 for c in contacts)
                and all(abs(c['cube_local_mm'][axis])<25 for c,sp in zip(contacts,specs) for axis in sp[-1])
                and min(v for _,v in clear)>.3 and selfmm<=1e-6 and supportmm<=.15 and pen<=1.)
            balanced=allocation=hold=None;force_ok=False
            if geometry_ok:
                balanced,allocation=allocate_native(m,d,cube,d.geom_xpos[cube],-m.body_mass[m.body('cube').id]*m.opt.gravity,
                    np.zeros(3),{'th_tip':2.,'if_tip':1.25,'mf_tip':1.25},hand,True,(nlo,nhi),
                    ('if_tip','mf_tip','th_tip'),normal_reference_weight=2.)
                if balanced is not None:
                    error=np.array(allocation['desired_wrench_tracking_residual']);goal=motor(balanced)
                    force_ok=bool(max(abs(error[:3]))<.005 and max(abs(error[3:]))<.0002
                        and np.all(goal>=lo) and np.all(goal<=hi)
                        and all(allocation['normal_N'].get(n,0)>.2 for n in ('if_tip','mf_tip','th_tip')))
                    if force_ok:
                        initial=dict(z);initial.update(qpos=d.qpos.copy(),qvel=np.zeros_like(d.qvel),
                            qacc_warmstart=np.zeros_like(d.qacc_warmstart),ctrl=ticks_to_sim(goal))
                        np.savez_compressed(folder/'independent_initial.npz',**initial)
                        np.savez_compressed(folder/'posture_candidate.npz',goal_ticks=goal,hardware_execution_allowed=False)
                        hold=run(m,initial,np.tile(goal,(134,1)),1,folder/'hold_end.npz',folder/'trace.npz',True,5,('if_tip','mf_tip','th_tip'))
            np.savez_compressed(folder/'static_proposal.npz',qpos=d.qpos,ctrl=d.ctrl)
            row=dict(name=name,geometry_passed=geometry_ok,force_allocation_passed=force_ok,contacts=contacts,
                upper_gap_mm=min(v for _,v in clear),self_mm=selfmm,support_mm=supportmm,cube_penetration_mm=pen,
                translation_mm=fit.x[-3:].tolist(),evaluations=fit.nfev,
                allocation=None if allocation is None else {k:v for k,v in allocation.items() if k!='contact_tasks'},hold=hold)
            cases.append(row);report=dict(status='bounded_three_finger_pose_preparation',model_sha256=sha(model),
                source_sha256=sha(source),parameters=vars(args),cases=cases,source_pose_initialization_is_not_a_reaching_path=True,
                continuous_cycle_verified=False,hardware_execution_allowed=False,devices_opened=False,elapsed_seconds=time.perf_counter()-began)
            (out/'review.json').write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps(dict(name=name,geometry=geometry_ok,force=force_ok,
                hold=None if hold is None else hold['whole_grip_task_passed'],upper_gap_mm=row['upper_gap_mm'],
                contacts=contacts,self_mm=selfmm)),flush=True)


if __name__=='__main__':main()
