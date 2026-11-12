"""Coordinate a declared MF push with measured TH load relief in the model.

Normal forces are hypotheses, not a calibration of physical motor current.
No devices, learning, changed geometry, or changed native safety limits.
"""
from pathlib import Path
import argparse
import json
import numpy as np
import mujoco
from scipy.optimize import lsq_linear
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,SIM_TO_MOTOR_IDS,TICK,save_json,sha
from directed_contact_state import forwarded_state,tip_yaw_contact,finger_yaw_moments
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate

def skew(v):
    x,y,z=v;return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])

def main():
    p=argparse.ArgumentParser();p.add_argument('--tag',required=True);p.add_argument('--source-state',required=True);p.add_argument('--steps',type=int,default=1000);p.add_argument('--counter-force',type=float,default=.2);a=p.parse_args()
    if not a.tag.replace('_','').isalnum() or not 1<=a.steps<=2400 or not 0<=a.counter_force<=1.:raise ValueError('Bounded local model comparison required')
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/a.tag;source=(OUT/a.source_state).resolve();geometry=base/'robust_mu025_feedback_gait_geometry_v8/review.json'
    if folder.exists() or not source.is_relative_to(OUT.resolve()):raise ValueError('New local study required')
    prior=json.loads(geometry.read_text());row=next(r for r in prior['rows'] if r['profile']=='axis2_sign-1');proposal=OUT/row['proposal'];assert sha(proposal)==row['proposal_sha256']
    m,initial,_,_,provenance=assets('active_ccd50');m.geom_friction[:,0]=.25;cube=m.geom('cube').id;mf=m.geom('mf_tip').id;th=m.geom('th_tip').id;active=np.r_[np.arange(4,8),np.arange(12,16)];sid=np.asarray(SIM_TO_MOTOR_IDS)-1
    ik=restore(m,dict(np.load(proposal,allow_pickle=False)));B=ik.geom_xmat[cube].reshape(3,3);C=ik.geom_xpos[cube];wanted=np.asarray(row['target_point_cube_frame_mm']);contact=min((c for c in ik.contact if {int(c.geom1),int(c.geom2)}=={cube,mf}),key=lambda c:np.linalg.norm((c.pos-C)@B*1000-wanted));R=ik.geom_xmat[mf].reshape(3,3);anchor=R.T@(contact.pos-ik.geom_xpos[mf]);local_normal=R.T@B[:,2];face=wanted.copy();face[2]=-27.5;face[:2]=np.clip(face[:2],-20,20)
    z=dict(np.load(source,allow_pickle=False));d=restore(m,z);g=Guard(m);previous=motor(d.ctrl);fraction=np.zeros(8);commands=[];trace=[];trajectory=[d.qpos.copy()];target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],180);phase='push';stop=[];rejections={};hold_count=held=clear=0;minimum=None;holdC=holdB=None;shift=drift=0.;cert=None;cert_enabled=False;samples=supported=0
    initial_moments={f:finger_yaw_moments(m,d,cube,f,initial['up']) for f in ('mf','th')}
    folder.mkdir();(folder/'source_servo_MF_with_TH_load_relief.py').write_bytes(Path(__file__).read_bytes())
    for step in range(a.steps):
        sensed=forwarded_state(m,d);B=sensed.geom_xmat[cube].reshape(3,3);C=sensed.geom_xpos[cube];error=orientation_error_deg(B,target);md=tip_yaw_contact(m,sensed,cube,mf,initial['up'],forward_copy=False)
        if phase=='push' and error<3 and face_up_error_deg(B,5,initial['up'])<5:phase='release'
        if phase=='release' and md['normal_N']<.03 and md['gap_mm']>.3:
            phase='hold';holdC=C.copy();holdB=B.copy();cert=SweepCertificate(m,g.hand,.3);lower,bad=cert.query(sensed,C,B);minimum=lower;cert_enabled=not bad and lower>.3
        if phase=='hold':command=previous.copy()
        else:
            R=sensed.geom_xmat[mf].reshape(3,3);point=sensed.geom_xpos[mf]+R@anchor;goal=C+B@face/1000;n=-B[:,2];delta=goal-point;normal_gap=float((point-goal)@n);tangent=delta-n*float(delta@n);tangent*=min(1.,.00008/max(np.linalg.norm(tangent),1e-12))
            if phase=='release':normal_step=float(np.clip(.0005-normal_gap,-.00008,.00008))
            elif normal_gap>.0003:normal_step=-min(normal_gap+.00002,.00008)
            else:normal_step=-float(np.clip((1.5-md['normal_N'])*.0001,-.000025,.000025))
            Jp=np.zeros((3,m.nv));Jr=np.zeros((3,m.nv));mujoco.mj_jac(m,sensed,Jp,Jr,point,int(m.geom_bodyid[mf]));tip_n=R@local_normal;Jn=-skew(tip_n)@Jr[:,active];normal_delta=(-n-tip_n)*.1;normal_delta*=min(1.,.02/max(np.linalg.norm(normal_delta),1e-12));A=[Jp[:,active]*TICK,Jn*TICK*.005];rhs=[tangent+n*normal_step,normal_delta*.005]
            # Move the loaded thumb along the CURRENT contact normal, avoiding
            # the incorrect face encountered by straight joint interpolation.
            contacts=[];local=np.zeros(6)
            for i,c in enumerate(sensed.contact):
                if cube not in (int(c.geom1),int(c.geom2)):continue
                other=int(c.geom2) if int(c.geom1)==cube else int(c.geom1)
                if not (m.body(int(m.geom_bodyid[other])).name or '').startswith('th_'):continue
                mujoco.mj_contactForce(m,sensed,i,local)
                if local[0]>.01:contacts.append((float(local[0]),c.pos.copy(),c.frame[:3].copy()*(1 if int(c.geom1)==cube else -1),int(m.geom_bodyid[other])))
            thumb_force=sum(c[0] for c in contacts)
            if contacts:
                force,point_t,normal_t,body=max(contacts,key=lambda c:c[0]);J=np.zeros((3,m.nv));mujoco.mj_jac(m,sensed,J,None,point_t,body)
                desired_force=0. if phase=='release' else a.counter_force;outward=float(np.clip((thumb_force-desired_force)*.0001,-.000015,.00004));A.append(J[:,active]*TICK);rhs.append(normal_t*outward)
            else:
                parked=np.zeros((4,8));parked[:,4:]=np.eye(4)*1e-5;A.append(parked);rhs.append(np.zeros(4))
            A.append(np.eye(8)*1e-7);rhs.append(np.zeros(8));lower=np.maximum(-2,np.ceil(g.lo[sid[active]])+4-previous[sid[active]]);upper=np.minimum(2,np.floor(g.hi[sid[active]])-4-previous[sid[active]]);lower=np.minimum(lower,2);upper=np.maximum(upper,-2);fixed=upper-lower<1e-12;lower[fixed]-=1e-10;upper[fixed]+=1e-10
            fit=lsq_linear(np.vstack(A),np.concatenate(rhs),bounds=(lower,upper),max_iter=100);fraction+=fit.x;candidate=previous.copy();candidate[sid[active]]=previous[sid[active]]+np.clip(np.rint(fraction),-2,2);candidate[sid[active]]=np.clip(candidate[sid[active]],np.minimum(previous[sid[active]]+2,np.ceil(g.lo[sid[active]])+4),np.maximum(previous[sid[active]]-2,np.floor(g.hi[sid[active]])-4));command=None
            for scale in (1.,.5,0.):
                trial=previous.copy();trial[sid[active]]=previous[sid[active]]+np.rint((candidate-previous)[sid[active]]*scale);branch=restore(m,{**{k:getattr(d,k).copy() for k in BOUND},'time':d.time});check=Guard(m);check.gap=g.gap;check.longest_gap=g.longest_gap;branch.ctrl[active]=ticks_to_sim(trial)[active];reason=[]
                for _ in range(75):
                    mujoco.mj_step(m,branch);reason=check.check(branch)
                    if np.linalg.norm(branch.qpos[-7:-4]-initial['initial_cube_center'])>.08:reason=list(reason)+['cube_translation_over80mm']
                    if phase=='push' and finger_yaw_moments(m,branch,cube,'mf',initial['up'])['net_direction_rejected']:reason=list(reason)+['nonpositive_MF_net_direction']
                    if reason:break
                if not reason:command=trial;break
                for name in reason:rejections[name]=rejections.get(name,0)+1
            if command is None:stop=['no_valid_two_finger_step'];break
            fraction-=command[sid[active]]-previous[sid[active]];fraction=np.clip(fraction,-.49,.49)
        assert np.max(abs(command-previous))<=2 and np.all(command>=g.lo) and np.all(command<=g.hi);d.ctrl[active]=ticks_to_sim(command)[active];commands.append(command.copy());previous=command.copy()
        for _ in range(75):
            mujoco.mj_step(m,d);stop=g.check(d);samples+=1;support=sample_model(m,d,g.hand,cube)['supported_sample'];supported+=support
            if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            if phase=='hold':
                hold_count+=1;held+=support;Bnow=d.geom_xmat[cube].reshape(3,3);Cnow=d.geom_xpos[cube];shift=max(shift,float(np.linalg.norm(Cnow-holdC)*1000));drift=max(drift,orientation_error_deg(Bnow,holdB))
                if cert_enabled:
                    lower,bad=cert.query(d,Cnow,Bnow);minimum=min(minimum,lower);clear+=not bad and lower>.3
                    if bad or lower<=.3:cert_enabled=False
            if stop:break
        trajectory.append(d.qpos.copy());now=forwarded_state(m,d);B=now.geom_xmat[cube].reshape(3,3);entry=dict(step=step+1,phase=phase,error180_deg=orientation_error_deg(B,target),MF=tip_yaw_contact(m,now,cube,mf,initial['up'],forward_copy=False),TH=finger_yaw_moments(m,now,cube,'th',initial['up']));trace.append(entry)
        if (step+1)%40==0:print(json.dumps(dict(step=step+1,phase=phase,error180_deg=entry['error180_deg'],TH_normal_moment_Nm=entry['TH']['normal_yaw_moment_Nm'])),flush=True)
        if stop or hold_count>=4050:break
    B=d.geom_xmat[cube].reshape(3,3);error=orientation_error_deg(B,target);tilt=face_up_error_deg(B,5,initial['up']);passed=bool(not stop and hold_count==clear==4050 and held/4050>.8 and shift<2 and drift<3 and error<5 and tilt<5)
    np.savez_compressed(folder/'research_goals.npz',start=motor(z['ctrl']),goals=np.asarray(commands,dtype=int).reshape(-1,16));np.savez_compressed(folder/'trajectory.npz',qpos=trajectory,control_seconds=.075);np.savez_compressed(folder/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
    r=dict(status='MF_push_with_actual_TH_normal_load_relief',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),geometry_sha256=sha(geometry),model_provenance=provenance,sliding_friction_hypothesis=.25,MF_pressure_hypothesis_N=1.5,TH_pressure_hypothesis_N=a.counter_force,model_forces_not_calibrated_to450mA=True,steps=len(commands),stop=stop,phase=phase,initial_finger_moments=initial_moments,trace=trace,final180_error_deg=error,tilt_deg=tilt,hold_samples=hold_count,supported_samples=held,upper_certified_samples=clear,minimum_upper_lower_mm=minimum,hold_shift_mm=shift,hold_drift_deg=drift,rotation_and_clear_hold_at_reached_location_passed=passed,samples=samples,whole_hand_support_fraction=supported/max(1,samples),original_native_guards_and_joint_limits_preserved=True,other8_joint_commands_unchanged=True,actual_state_resets_after_source=0,candidate_rejections=rejections,worst=g.worst,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False);save_json(folder/'review.json',r);print(json.dumps({k:v for k,v in r.items() if k not in ('trace','initial_finger_moments','model_provenance')}),flush=True)

if __name__=='__main__':main()
