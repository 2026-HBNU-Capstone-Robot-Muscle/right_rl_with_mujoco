"""Follow a declared cube-face point with MF; never close toward another face.

Uses privileged model geometry and force. This is an offline teacher, without
device access, SAC training, or a physical force/current calibration.
"""
from pathlib import Path
import argparse
import json
import numpy as np
import mujoco
from scipy.optimize import lsq_linear
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,SIM_TO_MOTOR_IDS,TICK,save_json,sha
from directed_contact_state import forwarded_state,tip_yaw_contact
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg,upright_yaw_target,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def skew(v):
    x,y,z=v;return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])


def main():
    p=argparse.ArgumentParser();p.add_argument('--tag',default='robust_mu025_declared_face_feedback_v4');p.add_argument('--steps',type=int,default=800);p.add_argument('--source-state');p.add_argument('--pressures',default='0.3,0.5');p.add_argument('--resume-controller');p.add_argument('--geometry');p.add_argument('--profile',default='axis2_sign-1');p.add_argument('--joint-reserve-ticks',type=int,default=0);a=p.parse_args()
    if not 0<=a.joint_reserve_ticks<=8:raise ValueError('Only inward command reserves are supported')
    if not a.tag.replace('_','').isalnum() or not 1<=a.steps<=3200:raise ValueError('Bounded local teacher study required')
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/a.tag;source=base/'reposition_then_second_rotation_20261111/free_standoff_replay_verification/end_state.npz';geometry=base/'robust_mu025_reposition_MF_gait2_geometry/review.json'
    if a.source_state:
        source=(OUT/a.source_state).resolve()
        if not source.is_relative_to(OUT.resolve()):raise ValueError('Local actual source required')
    if a.geometry:
        geometry=(OUT/a.geometry).resolve()
        if not geometry.is_relative_to(OUT.resolve()):raise ValueError('Local geometry study required')
    pressures=[float(p) for p in a.pressures.split(',')]
    if not 1<=len(pressures)<=3 or any(not .1<=p<=1. for p in pressures):raise ValueError('Bounded model pressure hypotheses required')
    resumed=None
    if a.resume_controller:
        controller=(OUT/a.resume_controller).resolve()
        if not controller.is_relative_to(OUT.resolve()) or source.resolve()!=controller.parent/'end_state.npz':raise ValueError('Controller must belong to the actual source state')
        resumed=dict(np.load(controller,allow_pickle=False))
        if len(pressures)!=1 or pressures[0]!=float(resumed['pressure']) or str(resumed['phase']) not in ('approach','push'):raise ValueError('Resume the same pressure before final release/hold')
    if folder.exists():raise RuntimeError('Preserve declared-face trials')
    prior=json.loads(geometry.read_text());row=next(r for r in prior['rows'] if r['profile']==a.profile);assert row['static_contact_candidate'] and row['finger']=='mf';proposal=OUT/row['proposal'];assert sha(proposal)==row['proposal_sha256']
    m,initial,_,_,provenance=assets('active_ccd50');m.geom_friction[:,0]=.25;cube=m.geom('cube').id;tip=m.geom('mf_tip').id;active=np.arange(4,8);sid=np.asarray(SIM_TO_MOTOR_IDS)-1
    ik=restore(m,dict(np.load(proposal,allow_pickle=False)));C=ik.geom_xpos[cube].copy();B=ik.geom_xmat[cube].reshape(3,3).copy();axis=int(a.profile[4]);sign=1 if '+' in a.profile else -1;normal=sign*B[:,axis];wanted=np.asarray(row['target_point_cube_frame_mm']);contacts=[c for c in ik.contact if {int(c.geom1),int(c.geom2)}=={cube,tip}];contact=min(contacts,key=lambda c:np.linalg.norm((c.pos-C)@B*1000-wanted));G=ik.geom_xmat[tip].reshape(3,3).copy();anchor=G.T@(contact.pos-ik.geom_xpos[tip]);local_normal=G.T@(-normal);face=wanted.copy();face[axis]=sign*27.5;across=2 if axis==0 else 0;face[across]=np.clip(face[across],-20,20);face[1]=np.clip(face[1],-20,20)
    z=dict(np.load(source,allow_pickle=False));targetB=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],180);folder.mkdir();(folder/'source_servo_declared_face_approach.py').write_bytes(Path(__file__).read_bytes());results=[]
    mf_bodies={i for i in range(m.nbody) if (m.body(i).name or '').startswith('mf_')}
    def opposite_loaded_contact(d):
        wrench=np.zeros(6)
        for i,c in enumerate(d.contact):
            if cube not in (int(c.geom1),int(c.geom2)):continue
            other=int(c.geom2) if int(c.geom1)==cube else int(c.geom1)
            if int(m.geom_bodyid[other]) not in mf_bodies:continue
            mujoco.mj_contactForce(m,d,i,wrench)
            if wrench[0]<.05:continue
            n=c.frame[:3]*(1 if int(c.geom1)==cube else -1);lever=float(np.cross(c.pos-d.geom_xpos[cube],-n)@initial['up']*1000)
            if lever<=0:return True
        return False
    for pressure in pressures:
        sub=folder/f'force{pressure:g}';sub.mkdir();d=restore(m,z);g=Guard(m);previous=motor(d.ctrl);fraction=np.zeros(4);commands=[];trajectory=[d.qpos.copy()];trace=[];stop=[];phase='approach';hold_count=held=clear=0;minimum=None;holdC=holdB=None;shift=drift=0.;cert=None;cert_enabled=False;candidate_rejections={};best=float('inf')
        if resumed is not None:
            fraction=resumed['fractional_ticks'].copy();phase=str(resumed['phase']);g.gap=float(resumed['guard_gap']);g.longest_gap=float(resumed['guard_longest_gap'])
        for step in range(a.steps):
            sensed=forwarded_state(m,d);curB=sensed.geom_xmat[cube].reshape(3,3);curC=sensed.geom_xpos[cube];error=orientation_error_deg(curB,targetB);best=min(best,error);direction=tip_yaw_contact(m,sensed,cube,tip,initial['up'],forward_copy=False)
            if phase in ('approach','push') and error<3 and face_up_error_deg(curB,5,initial['up'])<5:phase='release'
            if phase=='approach' and direction['positive_contact_ready'] and direction['normal_N']>.05:phase='push'
            if phase=='release' and direction['normal_N']<.03 and direction['gap_mm']>.3:
                phase='hold';holdC=curC.copy();holdB=curB.copy();cert=SweepCertificate(m,g.hand,.3);lower,bad=cert.query(sensed,curC,curB);minimum=lower;cert_enabled=not bad and lower>.3
            if phase=='hold':command=previous.copy()
            else:
                R=sensed.geom_xmat[tip].reshape(3,3);point=sensed.geom_xpos[tip]+R@anchor;face_world=curC+curB@face/1000;n=sign*curB[:,axis];tip_n=R@local_normal
                delta=face_world-point;normal_gap=float((point-face_world)@n);tangent=delta-n*float(delta@n);tangent*=min(1.,.00008/max(np.linalg.norm(tangent),1e-12))
                requested_force=0. if phase=='release' else pressure
                if phase=='release':normal_step=float(np.clip(.0005-normal_gap,-.00008,.00008))
                elif normal_gap>.0003:normal_step=-min(normal_gap+.00002,.00008)
                else:normal_step=-float(np.clip((requested_force-direction['normal_N'])*.0001,-.000025,.000025))
                desired_move=tangent+n*normal_step;Jp=np.zeros((3,m.nv));Jr=np.zeros((3,m.nv));mujoco.mj_jac(m,sensed,Jp,Jr,point,int(m.geom_bodyid[tip]));Jn=-skew(tip_n)@Jr[:,active]
                normal_delta=(-n-tip_n)*.1;normal_delta*=min(1.,.02/max(np.linalg.norm(normal_delta),1e-12));A=np.vstack([Jp[:,active]*TICK,Jn*TICK*.005,np.eye(4)*1e-7]);rhs=np.r_[desired_move,normal_delta*.005,np.zeros(4)]
                fit=lsq_linear(A,rhs,bounds=(-2,2),max_iter=100);fraction+=fit.x;change=np.clip(np.rint(fraction),-2,2);candidate=previous.copy()
                lower=np.minimum(previous[sid[active]]+2,np.ceil(g.lo[sid[active]])+a.joint_reserve_ticks);upper=np.maximum(previous[sid[active]]-2,np.floor(g.hi[sid[active]])-a.joint_reserve_ticks)
                candidate[sid[active]]=np.clip(previous[sid[active]]+change,lower,upper)
                accepted=None
                for scale in (1.,.5,0.):
                    trial=previous.copy();trial[sid[active]]=previous[sid[active]]+np.rint((candidate-previous)[sid[active]]*scale);branch=restore(m,{**{k:getattr(d,k).copy() for k in BOUND},'time':d.time});check=Guard(m);check.gap=g.gap;check.longest_gap=g.longest_gap;branch.ctrl[active]=ticks_to_sim(trial)[active];reason=[]
                    for _ in range(75):
                        mujoco.mj_step(m,branch);reason=check.check(branch)
                        if np.linalg.norm(branch.qpos[-7:-4]-initial['initial_cube_center'])>.08:reason=list(reason)+['cube_translation_over80mm']
                        if phase!='release' and opposite_loaded_contact(branch):reason=list(reason)+['opposite_MF_contact']
                        if reason:break
                    if not reason:accepted=trial;break
                    for why in reason:candidate_rejections[why]=candidate_rejections.get(why,0)+1
                if accepted is None:stop=['no_valid_declared_face_step'];break
                command=accepted;fraction-=command[sid[active]]-previous[sid[active]]
            assert np.max(abs(command-previous))<=2 and np.all(command>=g.lo) and np.all(command<=g.hi);d.ctrl[active]=ticks_to_sim(command)[active];commands.append(command.copy());previous=command.copy()
            for _ in range(75):
                mujoco.mj_step(m,d);stop=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if phase=='hold':
                    hold_count+=1;held+=sample_model(m,d,g.hand,cube)['supported_sample'];Bnow=d.geom_xmat[cube].reshape(3,3);Cnow=d.geom_xpos[cube];shift=max(shift,float(np.linalg.norm(Cnow-holdC)*1000));drift=max(drift,orientation_error_deg(Bnow,holdB))
                    if cert_enabled:
                        lower,bad=cert.query(d,Cnow,Bnow);minimum=min(minimum,lower);clear+=not bad and lower>.3
                        if bad or lower<=.3:cert_enabled=False
                if stop:break
            trajectory.append(d.qpos.copy());now=forwarded_state(m,d);trace.append(dict(step=step+1,phase=phase,error180_deg=orientation_error_deg(now.geom_xmat[cube].reshape(3,3),targetB),actual_contact=tip_yaw_contact(m,now,cube,tip,initial['up'],forward_copy=False)))
            if (step+1)%40==0:print(json.dumps(dict(pressure=pressure,**trace[-1])),flush=True)
            if stop or hold_count>=4050:break
        final=forwarded_state(m,d);Bfinal=final.geom_xmat[cube].reshape(3,3);error=orientation_error_deg(Bfinal,targetB);tilt=face_up_error_deg(Bfinal,5,initial['up']);passed=bool(not stop and hold_count==clear==4050 and held/4050>.8 and shift<2 and drift<3 and error<5 and tilt<5)
        np.savez_compressed(sub/'research_goals.npz',start=motor(z['ctrl']),goals=np.asarray(commands,dtype=int).reshape(-1,16));np.savez_compressed(sub/'trajectory.npz',qpos=trajectory,control_seconds=.075);np.savez_compressed(sub/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        np.savez_compressed(sub/'controller_state.npz',fractional_ticks=fraction,phase=phase,pressure=pressure,guard_gap=g.gap,guard_longest_gap=g.longest_gap)
        result=dict(path=sub.relative_to(OUT).as_posix(),pressure_hypothesis_N=pressure,steps=len(commands),stop=stop,phase=phase,best180_error_deg=min(best,error),final180_error_deg=error,tilt_deg=tilt,hold_samples=hold_count,supported_samples=held,upper_certified_samples=clear,minimum_upper_lower_mm=minimum,hold_shift_mm=shift,hold_drift_deg=drift,rotation_and_clear_hold_at_reached_location_passed=passed,candidate_rejections=candidate_rejections,actual_contact=tip_yaw_contact(m,final,cube,tip,initial['up'],forward_copy=False),trace=trace,worst=g.worst);results.append(result);save_json(sub/'review.json',result);print(json.dumps({k:v for k,v in result.items() if k!='trace'}),flush=True)
    save_json(folder/'review.json',dict(status='declared_moving_face_point_feedback_teacher',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),geometry=geometry.relative_to(OUT).as_posix(),geometry_sha256=sha(geometry),rows=results,sliding_friction_hypothesis=.25,declared_face_axis=axis,declared_face_sign=sign,declared_point_cube_frame_mm=face.tolist(),model_force_not_calibrated_to450mA=True,only_MF_commands_changed=True,original_native_guards_and_joint_limits_preserved=True,full_initial_placement_cycle_not_yet_replayed=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
