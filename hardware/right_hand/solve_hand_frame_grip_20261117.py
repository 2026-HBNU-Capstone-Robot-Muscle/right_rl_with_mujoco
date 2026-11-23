"""Supported front/back cage in the fixed hand's palm frame, offline only.

The target top face follows a real palm-box plane; it is explicitly distinct
from the old gravity-upright goal. Model/limits are unchanged. Both verdicts
are retained. No physical robot, camera, external cube forces or runtime API.
"""
from pathlib import Path
import argparse,hashlib,json,sys,time
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from opposed_grip_force_balance_20261116 import allocate
from canonical_grip_dynamics_20261105 import run
from hand_motor_mapping import TICK,ticks_to_sim


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--thumb-x',type=int,choices=(0,-8),default=0)
    ap.add_argument('--seed',choices=('source','open','curled'),default='source');a=ap.parse_args()
    tag=f'hand_frame_thumb{a.thumb_x}'+('_'+a.seed if a.seed!='source' else '')
    out=ROOT/'right_hand_rotation_20261117/opposed_lower_grip'/tag
    out.mkdir(parents=True,exist_ok=False)
    source=ROOT/'right_hand_rotation_20261117/opposed_lower_grip/shared_palm_station18/independent_initial.npz'
    model=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':model.read_bytes()});z=dict(np.load(source,allow_pickle=False));d=restore(m,z)
    hand,_,check=inspect_for(m);geometry=precompute(m,hand);vertices={g:v for g,v,e in geometry};cube=m.geom('cube').id
    palm=[m.geom(g).id for g in range(m.ngeom) if (m.geom(g).name or '').startswith('palm_collision_')]
    G=d.geom_xmat[m.geom('palm_collision_6').id].reshape(3,3)
    axis=int(np.argmax(abs(z['up']@G)));up=G[:,axis]*np.sign(z['up']@G[:,axis])
    down=m.opt.gravity-up*float(m.opt.gravity@up);down/=np.linalg.norm(down)
    B=np.column_stack([np.cross(-up,down),-up,down]);quat=Rotation.from_matrix(B).as_quat()
    base=d.qpos.copy();C0=d.geom_xpos[cube].copy()-up*.005
    lo,hi,nlo,nhi=native(m);active=np.r_[np.arange(8),np.arange(12,16)]
    lower=np.r_[nlo[active]+8*TICK,[-20]*3];upper=np.r_[nhi[active]-8*TICK,[20]*3]
    specs=[('if_tip',-B[:,2],np.arange(4),-18.,.8),('mf_tip',-B[:,2],np.arange(4,8),18.,.8),
           ('th_tip',B[:,2],np.arange(12,16),float(a.thumb_x),1.6)]
    began=time.perf_counter()
    def measure(x,exact=False):
        C=C0+x[-3:]/1000;d.qpos[:]=base;d.qpos[active]=x[:-3];d.qpos[-7:-4]=C;d.qpos[-4:]=np.r_[quat[3],quat[:3]]
        d.qvel[:]=0;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
        tasks=[];points=[];ctrl=[];taus=[];indices=[]
        for name,normal,j,target_x,force in specs:
            g=m.geom(name).id;world=vertices[g]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
            projected=(world-C)@normal;weight=np.exp(-(projected-projected.min())/.0002)
            point=np.average(world,axis=0,weights=weight);local=(point-C)@B*1000;gap=(projected.min()-.0275)*1000
            tasks.extend([(gap+.03)*10,local[0]-target_x,(local[1]-14)*2])
            jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,point,int(m.geom_bodyid[g]))
            tau=d.qfrc_bias[j]-d.qfrc_passive[j]-(normal@jp[:,j])*force
            ctrl.extend(d.qpos[j]+tau/m.actuator_gainprm[j,0]);taus.extend(tau);indices.extend(j)
            points.append(dict(name=name,gap_mm=float(gap),local_mm=local.tolist()))
        nearest=min(float(mujoco.mj_geomDistance(m,d,g,cube,.1,None)) for g in palm)*1000
        tasks.append((nearest+.02)*15)
        clear=gaps(m,d,C,B,geometry,exact);su,sp=check(d)
        pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
        return np.array(tasks),points,clear,max(sp.values(),default=0.),max(su.values(),default=0.),pen,np.array(ctrl),np.array(taus),np.array(indices)
    def residual(x):
        task,_,clear,selfmm,supportmm,pen,ctrl,tau,j=measure(x)
        return np.r_[task,np.maximum(.8-np.array([v for _,v in clear]),0)*10,
            np.maximum(nlo[j]+4*TICK-ctrl,0)*500,np.maximum(ctrl-nhi[j]+4*TICK,0)*500,
            np.maximum(abs(tau)-.15,0)*1000,(x[:-3]-base[active])*.1,x[-3:]*.15,
            selfmm*500000,supportmm*1000,max(0.,pen-.15)*1000]
    seed=np.r_[np.clip(base[active],lower[:-3]+1e-8,upper[:-3]-1e-8),[0,0,0]]
    if a.seed=='open':
        opened=ticks_to_sim(np.load(ROOT/'right_hand_rotation_20261027/loaded_grasp_recovery/open_path.npz')['open_goal'])
        seed[:-3]=np.clip(opened[active],lower[:-3]+1e-8,upper[:-3]-1e-8)
    elif a.seed=='curled':
        seed[:-3]=np.clip(np.r_[[.1,0.,.8,1.],[.1,0.,.8,1.],[.1,1.4,1.,.1]],lower[:-3]+1e-8,upper[:-3]-1e-8)
    fit=least_squares(residual,seed,bounds=(lower,upper),max_nfev=260,ftol=1e-9,xtol=1e-9,gtol=1e-9)
    _,points,clear,selfmm,supportmm,pen,_,_,_=measure(fit.x,True)
    actual=[];present=set();under=[]
    for c in d.contact:
        aa,bb=int(c.geom1),int(c.geom2)
        if cube not in (aa,bb):continue
        g=bb if aa==cube else aa;name=m.geom(g).name;local=(c.pos-d.geom_xpos[cube])@B*1000
        actual.append(dict(name=name,gap_mm=float(c.dist*1000),local_mm=local.tolist()))
        if int(m.geom_bodyid[g]) not in hand or c.dist>=.0004:continue
        if local[1]>-55/6:present.add(name)
        normal=c.frame[:3]*(1 if aa==cube else -1)
        if local[1]>20 and normal@B[:,1]>.5:under.append(name)
    roles=('if_tip','mf_tip','th_tip')
    geometry_ok=bool(set(roles)<=present and under and min(v for _,v in clear)>.3 and selfmm<=1e-6 and supportmm<=.15 and pen<=1.)
    balanced=allocation=hold=None;normal_certificate=None;assessment=None
    if geometry_ok:
        balanced,normal_certificate=allocate(m,d,cube,hand,nlo,nhi,lo,hi,required_tips=roles,normal_only=True)
        if balanced is not None:allocation=normal_certificate
        else:balanced,allocation=allocate(m,d,cube,hand,nlo,nhi,lo,hi,required_tips=roles)
        if balanced is not None:
            goal=motor(balanced);initial=dict(z);initial.update(qpos=d.qpos.copy(),qvel=np.zeros_like(d.qvel),qacc_warmstart=np.zeros_like(d.qacc_warmstart),ctrl=ticks_to_sim(goal))
            initial['initial_cube_center']=d.geom_xpos[cube].copy();initial['initial_cube_rotation']=B.copy()
            np.savez_compressed(out/'independent_initial.npz',**initial)
            np.savez_compressed(out/'posture_candidate.npz',goal_ticks=goal,hardware_execution_allowed=False)
            hold=run(m,initial,np.tile(goal,(534,1)),1,out/'hold_end.npz',out/'trace.npz',True,5,roles)
            trace=np.load(out/'trace.npz',allow_pickle=False);measure_d=mujoco.MjData(m);max_angle=max_shift=0.
            for q in trace['qpos']:
                measure_d.qpos[:]=q;mujoco.mj_forward(m,measure_d)
                rotation=measure_d.geom_xmat[cube].reshape(3,3)
                angle=float(np.degrees(np.linalg.norm(Rotation.from_matrix(rotation@B.T).as_rotvec())))
                max_angle=max(max_angle,angle);max_shift=max(max_shift,float(np.linalg.norm(measure_d.geom_xpos[cube]-initial['initial_cube_center'])*1000))
            end_rotation=np.array(hold['final_rotation_world'])
            target_error=float(np.degrees(np.linalg.norm(Rotation.from_matrix(end_rotation@B.T).as_rotvec())))
            tail=hold['settled_tail'];assessment=dict(working_frame='fixed palm plane, front face downhill',
                target_tilt_from_gravity_deg=float(np.degrees(np.arccos(np.clip(up@z['up'],-1,1)))),
                final_target_error_deg=target_error,maximum_sampled_angle_deg=max_angle,maximum_sampled_shift_mm=max_shift,
                evaluated_hold_seconds=40.05,original_gravity_upright_verdict=hold['whole_grip_task_passed'])
            assessment['common_hand_frame_grip_passed']=bool(hold['safety_passed'] and hold['completed_goals']==534
                and target_error<5 and max_angle<5 and max_shift<2 and tail['minimum_upper_gap_sampled_75ms_mm']>.3
                and tail['separation_certificate_bad_steps_1ms']==0 and tail['opposed_lower_body_contact_fraction']>.8)
    np.savez_compressed(out/'static_proposal.npz',qpos=d.qpos,ctrl=d.ctrl)
    report=dict(status='fixed_hand_frame_supported_opposed_grip',source_sha256=sha(source),model_sha256=sha(model),source_script_sha256=sha(Path(__file__)),
        parameters=vars(a),target_rotation_world=B.tolist(),gravity_direction_preserved=m.opt.gravity.tolist(),geometry_passed=geometry_ok,
        proxy_points=points,actual_contacts=actual,under_support=under,upper_gap_mm=min(v for _,v in clear),self_mm=selfmm,support_mm=supportmm,cube_mm=pen,
        translation_mm=fit.x[-3:].tolist(),normal_only_force_certificate=normal_certificate,allocation=allocation,hold=hold,assessment=assessment,
        gravity_upright_task_not_relabelled=True,left_hand_alignment_verified=False,continuous_cycle_verified=False,
        requested_hold_seconds=40.05,hardware_execution_allowed=False,devices_opened=False,elapsed_seconds=time.perf_counter()-began)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(geometry=geometry_ok,force=balanced is not None,points=points,under=under,
        upper_gap_mm=report['upper_gap_mm'],assessment=assessment,
        hold=None if hold is None else {k:hold[k] for k in ['safety_passed','settled_tail','stop']})),flush=True)


if __name__=='__main__':main()
