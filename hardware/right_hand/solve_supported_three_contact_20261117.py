"""Place the cube over the shared workspace of index/middle and thumb.

Any actual palm underside contact can support weight. No particular palm box
or finger link is mandatory. Static proposals are independently replayed free.
"""
from pathlib import Path
import argparse,hashlib,json,sys,time
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from scipy.optimize import least_squares
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from opposed_grip_force_balance_20261116 import allocate
from canonical_grip_dynamics_20261105 import run
from hand_motor_mapping import TICK,ticks_to_sim


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--station-mm',type=int,choices=(18,26),default=18);a=ap.parse_args()
    out=ROOT/'right_hand_rotation_20261117/opposed_lower_grip'/f'shared_palm_station{a.station_mm}'
    out.mkdir(parents=True,exist_ok=False)
    source=ROOT/'right_hand_rotation_20261116/opposed_lower_grip/independent_initial.npz'
    model=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':model.read_bytes()});z=dict(np.load(source,allow_pickle=False));d=restore(m,z)
    hand,_,check=inspect_for(m);geometry=precompute(m,hand);vertices={g:v for g,v,e in geometry}
    cube=m.geom('cube').id;C0=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();base=d.qpos.copy()
    lo,hi,nlo,nhi=native(m);active=np.r_[np.arange(8),np.arange(12,16)]
    lower=np.r_[nlo[active]+8*TICK,[-30]*3];upper=np.r_[nhi[active]-8*TICK,[30]*3]
    desired=B[:,0]*a.station_mm
    palm=[m.geom(name).id for name in [m.geom(g).name for g in range(m.ngeom)] if name and name.startswith('palm_collision_')]
    specs=[('if_tip',-B[:,2],np.arange(4),-18.,1.2),
           ('mf_tip',-B[:,2],np.arange(4,8),18.,1.2),
           ('th_tip',B[:,2],np.arange(12,16),0.,2.)]
    began=time.perf_counter()
    def measure(x,exact=False):
        C=C0+x[-3:]/1000;d.qpos[:]=base;d.qpos[active]=x[:-3];d.qpos[-7:-4]=C;d.qvel[:]=0;d.ctrl[:]=d.qpos[:16]
        mujoco.mj_forward(m,d);task=[];points=[];ctrl=[];taus=[];indices=[]
        for name,normal,j,target_x,force in specs:
            g=m.geom(name).id;world=vertices[g]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
            projected=(world-C)@normal;weight=np.exp(-(projected-projected.min())/.0002)
            point=np.average(world,axis=0,weights=weight);local=(point-C)@B*1000;gap=(projected.min()-.0275)*1000
            task.extend([(gap+.03)*10,local[0]-target_x,(local[1]-14)*2])
            jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,point,int(m.geom_bodyid[g]))
            tau=d.qfrc_bias[j]-d.qfrc_passive[j]-(normal@jp[:,j])*force
            ctrl.extend(d.qpos[j]+tau/m.actuator_gainprm[j,0]);taus.extend(tau);indices.extend(j)
            points.append(dict(name=name,gap_mm=float(gap),local_mm=local.tolist()))
        closest=min(float(mujoco.mj_geomDistance(m,d,g,cube,.1,None)) for g in palm)*1000
        task.append((closest+.02)*15)
        clear=gaps(m,d,C,B,geometry,exact);su,sp=check(d)
        pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
        return np.array(task),points,clear,max(sp.values(),default=0.),max(su.values(),default=0.),pen,np.array(ctrl),np.array(taus),np.array(indices)
    def residual(x):
        task,_,clear,selfmm,supportmm,pen,ctrl,tau,j=measure(x)
        return np.r_[task,np.maximum(.8-np.array([v for _,v in clear]),0)*10,
            np.maximum(nlo[j]+4*TICK-ctrl,0)*500,np.maximum(ctrl-nhi[j]+4*TICK,0)*500,
            np.maximum(abs(tau)-.15,0)*1000,(x[:-3]-base[active])*.1,(x[-3:]-desired)*.2,
            selfmm*500000,supportmm*1000,max(0.,pen-.15)*1000]
    seed=np.r_[np.clip(base[active],lower[:-3]+1e-8,upper[:-3]-1e-8),desired]
    fit=least_squares(residual,seed,bounds=(lower,upper),max_nfev=240,ftol=1e-9,xtol=1e-9,gtol=1e-9)
    task,points,clear,selfmm,supportmm,pen,_,_,_=measure(fit.x,True)
    actual=[];present=set();under=[]
    for c in d.contact:
        aa,bb=int(c.geom1),int(c.geom2)
        if cube not in (aa,bb):continue
        other=bb if aa==cube else aa;local=(c.pos-d.geom_xpos[cube])@B*1000;name=m.geom(other).name
        actual.append(dict(name=name,gap_mm=float(c.dist*1000),local_mm=local.tolist()))
        if int(m.geom_bodyid[other]) not in hand or c.dist>=.0004:continue
        if local[1]>-55/6:present.add(name)
        normal=c.frame[:3]*(1 if aa==cube else -1)
        if local[1]>20 and normal@B[:,1]>.5:under.append(name)
    required=('if_tip','mf_tip','th_tip')
    geometry_ok=bool(set(required)<=present and under and min(v for _,v in clear)>.3 and selfmm<=1e-6 and supportmm<=.15 and pen<=1.)
    balanced=allocation=hold=None
    if geometry_ok:
        balanced,allocation=allocate(m,d,cube,hand,nlo,nhi,lo,hi,required_tips=required)
        if balanced is not None:
            goal=motor(balanced);initial=dict(z);initial.update(qpos=d.qpos.copy(),qvel=np.zeros_like(d.qvel),qacc_warmstart=np.zeros_like(d.qacc_warmstart),ctrl=ticks_to_sim(goal))
            initial['initial_cube_center']=d.geom_xpos[cube].copy();initial['initial_cube_rotation']=d.geom_xmat[cube].reshape(3,3).copy()
            np.savez_compressed(out/'independent_initial.npz',**initial)
            np.savez_compressed(out/'posture_candidate.npz',goal_ticks=goal,hardware_execution_allowed=False)
            hold=run(m,initial,np.tile(goal,(534,1)),1,out/'hold_end.npz',out/'trace.npz',True,5,required)
    np.savez_compressed(out/'static_proposal.npz',qpos=d.qpos,ctrl=d.ctrl)
    report=dict(status='shared_palm_three_finger_grip',model_sha256=sha(model),source_sha256=sha(source),source_script_sha256=sha(Path(__file__)),
        geometry_passed=geometry_ok,proxy_points=points,actual_contacts=actual,under_support=under,upper_gap_mm=min(v for _,v in clear),
        self_mm=selfmm,support_mm=supportmm,cube_mm=pen,translation_mm=fit.x[-3:].tolist(),allocation=allocation,hold=hold,
        requested_hold_seconds=40.05,devices_opened=False,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(geometry=geometry_ok,points=points,under=under,upper_gap_mm=report['upper_gap_mm'],self_mm=selfmm,
        force=balanced is not None,hold=None if hold is None else {k:hold[k] for k in ['whole_grip_task_passed','final_up_tilt_deg','settled_tail','stop']})),flush=True)


if __name__=='__main__':main()
