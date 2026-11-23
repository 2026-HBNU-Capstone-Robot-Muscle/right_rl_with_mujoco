"""Align existing thumb/index pad faces, rather than pinching at corners.

Static IK uses existing collision meshes and limits. Only an independent,
unconstrained cube hold may establish success. This file has no hardware API.
"""
from pathlib import Path
import argparse,hashlib,json,sys,time
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.spatial import ConvexHull
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from opposed_grip_force_balance_20261116 import allocate
from canonical_grip_dynamics_20261105 import run
from hand_motor_mapping import TICK,ticks_to_sim


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def pad_face(vertices,wanted):
    hull=ConvexHull(vertices);groups={}
    for triangle,equation in zip(hull.simplices,hull.equations):
        points=vertices[triangle];area=np.linalg.norm(np.cross(points[1]-points[0],points[2]-points[0]))/2
        key=tuple(np.round(equation,7))
        group=groups.setdefault(key,dict(normal=equation[:3],area=0.,points=[]))
        group['area']+=area;group['points'].extend(triangle.tolist())
    candidates=[]
    for group in groups.values():
        alignment=float(group['normal']@wanted)
        if alignment<=.5 or group['area']<15e-6:continue
        group['center']=vertices[np.unique(group['points'])].mean(0)
        group['score']=group['area']*alignment**8;candidates.append(group)
    if not candidates:raise RuntimeError('No broad existing pad face points toward cube')
    return max(candidates,key=lambda p:p['score'])


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--normal-weight',type=int,choices=(30,60),default=30);a=ap.parse_args()
    out=ROOT/'right_hand_rotation_20261117/opposed_lower_grip'/f'flat_pads_w{a.normal_weight}'
    out.mkdir(parents=True,exist_ok=False)
    source=ROOT/'right_hand_rotation_20261116/opposed_lower_grip/index_base_support/palm_offset/independent_initial.npz'
    model=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':model.read_bytes()});z=dict(np.load(source,allow_pickle=False));d=restore(m,z)
    hand,_,check=inspect_for(m);geometry=precompute(m,hand);vertices={g:v for g,v,e in geometry}
    cube=m.geom('cube').id;C0=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();base=d.qpos.copy()
    lo,hi,nlo,nhi=native(m);active=np.r_[np.arange(4),np.arange(12,16)]
    lower=np.r_[nlo[active]+8*TICK,[-12]*3];upper=np.r_[nhi[active]-8*TICK,[12]*3]
    specs=[]
    for name,normal,j in [('if_tip',-B[:,2],np.arange(4)),('th_tip',B[:,2],np.arange(12,16))]:
        g=m.geom(name).id;G=d.geom_xmat[g].reshape(3,3);pad=pad_face(vertices[g],G.T@(-normal))
        specs.append((name,g,normal,j,pad))
    began=time.perf_counter()
    def measure(x,exact=False):
        C=C0+x[-3:]/1000;d.qpos[:]=base;d.qpos[active]=x[:-3];d.qpos[-7:-4]=C;d.qvel[:]=0;d.ctrl[:]=d.qpos[:16]
        mujoco.mj_forward(m,d);task=[];details=[];ctrl=[];taus=[];indices=[]
        for name,g,normal,j,pad in specs:
            G=d.geom_xmat[g].reshape(3,3);point=d.geom_xpos[g]+G@pad['center'];local=(point-C)@B*1000
            gap=(float((point-C)@normal)-.0275)*1000;outward=G@pad['normal']
            task.extend([(gap+.03)*10,(local[0]+18),(local[1]-14)*2]);task.extend((outward+normal)*a.normal_weight)
            jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,point,int(m.geom_bodyid[g]))
            tau=d.qfrc_bias[j]-d.qfrc_passive[j]-(normal@jp[:,j])*1.2
            ctrl.extend(d.qpos[j]+tau/m.actuator_gainprm[j,0]);taus.extend(tau);indices.extend(j)
            details.append(dict(name=name,gap_mm=gap,local_mm=local.tolist(),
                pad_area_mm2=pad['area']*1e6,pad_misalignment_deg=float(np.degrees(np.arccos(np.clip(outward@(-normal),-1,1))))))
        palm_gap=float(mujoco.mj_geomDistance(m,d,m.geom('palm_collision_1').id,cube,.1,None))*1000
        task.append((palm_gap+.02)*15)
        clear=gaps(m,d,C,B,geometry,exact);su,sp=check(d)
        pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
        return np.array(task),details,clear,max(sp.values(),default=0.),max(su.values(),default=0.),pen,np.array(ctrl),np.array(taus),np.array(indices)
    def residual(x):
        task,_,clear,selfmm,supportmm,pen,ctrl,tau,j=measure(x)
        return np.r_[task,np.maximum(.8-np.array([v for _,v in clear]),0)*10,
            np.maximum(nlo[j]+4*TICK-ctrl,0)*500,np.maximum(ctrl-nhi[j]+4*TICK,0)*500,
            np.maximum(abs(tau)-.15,0)*1000,(x[:-3]-base[active])*.1,x[-3:]*.12,
            selfmm*500000,supportmm*1000,max(0.,pen-.15)*1000]
    seed=np.r_[np.clip(base[active],lower[:-3]+1e-8,upper[:-3]-1e-8),[0,0,0]]
    fit=least_squares(residual,seed,bounds=(lower,upper),max_nfev=220,ftol=1e-9,xtol=1e-9,gtol=1e-9)
    task,details,clear,selfmm,supportmm,pen,_,_,_=measure(fit.x,True)
    actual=[]
    for c in d.contact:
        aa,bb=int(c.geom1),int(c.geom2)
        if cube not in (aa,bb):continue
        g=bb if aa==cube else aa
        actual.append(dict(name=m.geom(g).name,gap_mm=float(c.dist*1000),local_mm=((c.pos-d.geom_xpos[cube])@B*1000).tolist()))
    geometry_ok=bool(all(any(c['name']==name and c['gap_mm']<.4 for c in actual) for name in ('if_tip','th_tip'))
        and min(v for _,v in clear)>.3 and selfmm<=1e-6 and supportmm<=.15 and pen<=1.)
    balanced=allocation=hold=None
    if geometry_ok:
        balanced,allocation=allocate(m,d,cube,hand,nlo,nhi,lo,hi)
        if balanced is not None:
            goal=motor(balanced);initial=dict(z);initial.update(qpos=d.qpos.copy(),qvel=np.zeros_like(d.qvel),qacc_warmstart=np.zeros_like(d.qacc_warmstart),ctrl=ticks_to_sim(goal))
            np.savez_compressed(out/'independent_initial.npz',**initial)
            np.savez_compressed(out/'posture_candidate.npz',goal_ticks=goal,hardware_execution_allowed=False)
            hold=run(m,initial,np.tile(goal,(134,1)),1,out/'hold_end.npz',out/'trace.npz',True,5,('if_tip','th_tip'))
    np.savez_compressed(out/'static_proposal.npz',qpos=d.qpos,ctrl=d.ctrl)
    report=dict(status='existing_pad_face_alignment_trial',model_sha256=sha(model),source_sha256=sha(source),
        source_script_sha256=sha(Path(__file__)),geometry_passed=geometry_ok,details=details,actual_contacts=actual,
        upper_gap_mm=min(v for _,v in clear),self_mm=selfmm,support_mm=supportmm,cube_mm=pen,
        translation_mm=fit.x[-3:].tolist(),allocation=allocation,hold=hold,hardware_execution_allowed=False,
        devices_opened=False,elapsed_seconds=time.perf_counter()-began)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(geometry=geometry_ok,details=details,self_mm=selfmm,upper_gap_mm=report['upper_gap_mm'],
        force=balanced is not None,hold=None if hold is None else {k:hold[k] for k in ['whole_grip_task_passed','final_up_tilt_deg','settled_tail','stop']})),flush=True)


if __name__=='__main__':main()
