"""Thumb/index opposed lower grip with an index-base underside support.

Offline static proposals and independent free-cube holds only. No transport,
servo settings, cube welds or external forces. Existing source files untouched.
"""
from pathlib import Path
import argparse,hashlib,json,sys,time
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import mujoco,numpy as np
from scipy.optimize import least_squares
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from research_quasistatic_wrench_path_20261105 import allocate_native
from canonical_grip_dynamics_20261105 import run
from hand_motor_mapping import TICK,ticks_to_sim


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',choices=('equal14','thumb20','both20','index6','index6_thumb20','palm_offset','palm_offset_index6'),required=True)
    args=parser.parse_args()
    out=ROOT/'right_hand_rotation_20261116/opposed_lower_grip/index_base_support'/args.case
    out.mkdir(parents=True,exist_ok=False)
    source=out.parents[1]/'independent_initial.npz'
    model=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':model.read_bytes()})
    z=dict(np.load(source,allow_pickle=False));d=restore(m,z)
    base=d.qpos.copy();C0=d.geom_xpos[m.geom('cube').id].copy();B=d.geom_xmat[m.geom('cube').id].reshape(3,3).copy()
    lo,hi,nlo,nhi=native(m);active=np.r_[np.arange(4),np.arange(12,16)]
    offset_case=args.case.startswith('palm_offset')
    span=25 if offset_case else 12
    low=np.r_[nlo[active]+8*TICK,[-span]*3];high=np.r_[nhi[active]-8*TICK,[span]*3]
    hand,_,check=inspect_for(m);geometry=precompute(m,hand);vertices={g:v for g,v,e in geometry}
    heights={'equal14':(14.,14.),'thumb20':(14.,20.),'both20':(20.,20.),'index6':(6.,14.),'index6_thumb20':(6.,20.),'palm_offset':(14.,14.),'palm_offset_index6':(6.,14.)}[args.case]
    pressure=1.2 if offset_case else 2.
    specs=[('if_tip',-B[:,2],np.arange(4),pressure,heights[0]),
           ('th_tip',B[:,2],np.arange(12,16),pressure,heights[1])]
    if not offset_case:specs.append(('if_bs_collision_1',B[:,1],np.arange(4),.98,None))
    cube=m.geom('cube').id;began=time.perf_counter()
    def measure(x,exact=False):
        C=C0+x[-3:]/1000;d.qpos[:]=base;d.qpos[active]=x[:-3];d.qpos[-7:-4]=C
        d.qvel[:]=0;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
        tasks=[];points=[];ctrls=[];taus=[];indices=[]
        for name,normal,j,force,height in specs:
            g=m.geom(name).id;world=vertices[g]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
            projected=(world-C)@normal;weights=np.exp(-(projected-projected.min())/.0002)
            point=np.average(world,axis=0,weights=weights);local=(point-C)@B*1000
            gap=(projected.min()-.0275)*1000
            if height is None:
                tasks.extend([(gap+.02)*20,max(0.,abs(local[0])-22)*10,max(0.,abs(local[2])-22)*10])
            else:tasks.extend([(gap+.03)*10,(local[0]+18.) if offset_case else local[0]*.2,(local[1]-height)*2])
            jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,point,int(m.geom_bodyid[g]))
            tau=d.qfrc_bias[j]-d.qfrc_passive[j]-(normal@jp[:,j])*force
            ctrls.extend(d.qpos[j]+tau/m.actuator_gainprm[j,0]);taus.extend(tau);indices.extend(j)
            points.append(dict(name=name,gap_mm=float(gap),local_mm=local.tolist()))
        if offset_case:
            distance=float(mujoco.mj_geomDistance(m,d,m.geom('palm_collision_1').id,cube,.1,None))
            tasks.append((distance*1000+.02)*15)
        clear=gaps(m,d,C,B,geometry,exact);su,sp=check(d)
        pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
        return np.array(tasks),points,clear,max(sp.values(),default=0.),max(su.values(),default=0.),pen,np.array(ctrls),np.array(taus),np.array(indices)
    def residual(x):
        task,_,clear,selfmm,supportmm,pen,ctrl,tau,j=measure(x)
        return np.r_[task,np.maximum(.8-np.array([v for _,v in clear]),0)*10,
            np.maximum(nlo[j]+4*TICK-ctrl,0)*500,np.maximum(ctrl-nhi[j]+4*TICK,0)*500,
            np.maximum(abs(tau)-.15,0)*1000,(x[:-3]-base[active])*.1,x[-3:]*.12,
            selfmm*500000,supportmm*1000,max(0.,pen-.8)*1000]
    seed=np.r_[np.clip(base[active],low[:-3]+1e-8,high[:-3]-1e-8),B[:,0]*18 if offset_case else [0,0,0]]
    fit=least_squares(residual,seed,bounds=(low,high),max_nfev=220,ftol=1e-9,xtol=1e-9,gtol=1e-9)
    task,points,clear,selfmm,supportmm,pen,ctrl,tau,j=measure(fit.x,True)
    actual=[];present=set();underside=[]
    for c in d.contact:
        if cube not in (int(c.geom1),int(c.geom2)):continue
        other=int(c.geom2) if int(c.geom1)==cube else int(c.geom1);name=m.geom(other).name
        local=(c.pos-C0-fit.x[-3:]/1000)@B*1000
        actual.append(dict(name=name,gap_mm=float(c.dist*1000),local_mm=local.tolist()))
        if c.dist<.0004 and local[1]>-55/6:present.add(name)
        outward=c.frame[:3]*(1 if int(c.geom1)==cube else -1)
        if int(m.geom_bodyid[other]) in hand and c.dist<.0004 and local[1]>20 and outward@B[:,1]>.5:underside.append(m.geom(other).name)
    required={'if_tip','th_tip'} if offset_case else {'if_tip','th_tip','if_bs_collision_1'}
    geometry_ok=bool(required<=present and (not offset_case or underside) and min(v for _,v in clear)>.3 and selfmm<=1e-6 and supportmm<=.15 and pen<=1.)
    np.savez_compressed(out/'static_proposal.npz',qpos=d.qpos,ctrl=d.ctrl)
    allocation=None;force_ok=False;hold=None
    if geometry_ok:
        if offset_case:
            from opposed_grip_force_balance_20261116 import allocate
            balanced,allocation=allocate(m,d,cube,hand,nlo,nhi,lo,hi)
        else:
            balanced,allocation=allocate_native(m,d,cube,d.geom_xpos[cube],-m.body_mass[m.body('cube').id]*m.opt.gravity,
                np.zeros(3),{'if_tip':2.,'th_tip':2.,'if_bs_collision_1':.98},hand,True,(nlo,nhi),
                ('if_tip','th_tip','if_bs_collision_1'),normal_reference_weight=2.)
        if balanced is not None:
            goal=motor(balanced);err=np.array(allocation['exact_wrench_residual'] if offset_case else allocation['desired_wrench_tracking_residual'])
            force_ok=bool(max(abs(err[:3]))<.005 and max(abs(err[3:]))<.0002 and np.all(goal>=lo) and np.all(goal<=hi))
            if force_ok:
                initial=dict(z);initial.update(qpos=d.qpos.copy(),qvel=np.zeros_like(d.qvel),
                    qacc_warmstart=np.zeros_like(d.qacc_warmstart),ctrl=ticks_to_sim(goal))
                np.savez_compressed(out/'independent_initial.npz',**initial)
                np.savez_compressed(out/'posture_candidate.npz',goal_ticks=goal,hardware_execution_allowed=False)
                hold=run(m,initial,np.tile(goal,(134,1)),1,out/'hold_end.npz',out/'trace.npz',True,5,('if_tip','th_tip'))
    report=dict(status='index_base_supported_opposed_grip',case=args.case,source_sha256=sha(source),model_sha256=sha(model),
        geometry_passed=geometry_ok,force_passed=force_ok,actual_contacts=actual,proxy_points=points,
        upper_gap_mm=min(v for _,v in clear),self_mm=selfmm,support_mm=supportmm,cube_mm=pen,
        translation_mm=fit.x[-3:].tolist(),hold=hold,allocation=None if allocation is None else {k:v for k,v in allocation.items() if k!='contact_tasks'},
        devices_opened=False,hardware_execution_allowed=False,reaching_path_verified=False,elapsed_seconds=time.perf_counter()-began)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(case=args.case,geometry=geometry_ok,force=force_ok,upper_gap_mm=report['upper_gap_mm'],
        self_mm=selfmm,points=points,hold=None if hold is None else {k:hold[k] for k in ['whole_grip_task_passed','final_up_tilt_deg','settled_tail','stop']})),flush=True)


if __name__=='__main__':main()
