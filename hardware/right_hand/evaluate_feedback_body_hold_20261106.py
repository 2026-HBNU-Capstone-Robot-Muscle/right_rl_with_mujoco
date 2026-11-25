"""Bounded pose-feedback hold under an unobserved simulated layer reaction.

The controller reads simulated pose/contact data, never the applied load.
Native point-force allocation is converted into complete position goals,
including tangential force components. All original hand guards and2tick
steps remain. No real current/PID changes or device adapters are provided.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import restore,inspect_for,native,angle,motor
from hand_motor_mapping import TICK,SIM_TO_MOTOR_IDS,ticks_to_sim
from research_quasistatic_wrench_path_20261105 import allocate_native,hybrid_increment
from offline_fractional_goal_accumulator_20261105 import FractionalGoals
from upper_layer_separation_certificate_20261105 import SweepCertificate
from evaluate_canonical_grip_reaction_20261105 import load_fraction
from cube_task_frame_20261105 import orientation_error_deg

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
PROFILES={'baseline':(1.,1.),'gentle_fast':(.6,2.),'gentle_faster':(.6,4.),'medium_fast':(.8,2.),'low_fast':(.4,2.),'low_slow':(.5,1.),'gentle_slow':(.6,1.)}

def trial(m,z,torque,mode,out,profile='baseline'):
    d=restore(m,z);hand,support,check=inspect_for(m);cube=m.geom('cube').id;body=m.body('cube').id;C0=d.qpos[-7:-4].copy();B0=d.xmat[body].reshape(3,3).copy();axis=-B0[:,1];lo,hi,nlo,nhi=native(m);start=motor(d.ctrl);planner=FractionalGoals(start,np.minimum(lo+4,start),np.maximum(hi-4,start));reference={'if_tip':1.,'th_tip':1.};integral=np.zeros(6);ids=np.asarray(SIM_TO_MOTOR_IDS)-1;cert=SweepCertificate(m,hand,.3);gap=longest=0.;minclear=float('inf');cert_bad=cert_skipped=count=lowergood=0;maxyaw=maxshift=maxtilt=maxpose=0.;worst=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,range_rad=0.);rows=[];trace=[];stop=None;began=time.perf_counter()
    normal,pose_scale=PROFILES[profile];reference.update(if_tip=normal,th_tip=normal)
    for sample in range(134):
        C=d.qpos[-7:-4].copy();B=d.xmat[body].reshape(3,3);error=np.r_[C0-C,Rotation.from_matrix(B0@B.T).as_rotvec()];integral+=error*.075;integral[:3]=np.clip(integral[:3],-.002,.002);integral[3:]=np.clip(integral[3:],-.04,.04);velocity=np.zeros(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,body,velocity,0);F=-m.body_mass[body]*m.opt.gravity+100*error[:3]+20*integral[:3]-2*velocity[3:];T=.8*error[3:]+.2*integral[3:]-.008*velocity[:3];proposal,allocation=allocate_native(m,d,cube,C,F,T,reference,hand,True,(nlo,nhi),('if_tip','th_tip'),normal_reference_weight=2.)
        if proposal is None:stop='force_allocation:'+allocation['reason'];break
        if mode=='force':
            # Convert the allocated complete actuator torque to position goals.
            # The model's own actuator velocity bias is compensated here.
            proposal=proposal-m.actuator_biasprm[:,2]*d.qvel[:16]/m.actuator_gainprm[:,0];proposal[8:12]=d.ctrl[8:12];wanted=np.empty(16);wanted[ids]=proposal/TICK+2048;lower,upper=planner.increment_bounds();goals=planner.advance(np.clip(wanted-planner.continuous,lower,upper))
        else:
            targetC=C+(C0-C)*pose_scale
            targetB=Rotation.from_rotvec(Rotation.from_matrix(B0@B.T).as_rotvec()*pose_scale).as_matrix()@B
            goals=hybrid_increment(m,d,allocation,planner,C,targetC,B,targetB,maximum_tracking_rad=.12)
        d.ctrl[:]=ticks_to_sim(goals);rows.append(goals.copy())
        for sub in range(75):
            seconds=count*.001;d.xfrc_applied[body,3:]=axis*torque*load_fraction(seconds);mujoco.mj_step(m,d);count+=1;su,sp=check(d);worst['self_mm']=max(worst['self_mm'],max(sp.values(),default=0.));worst['support_mm']=max(worst['support_mm'],max(su.values(),default=0.));worst['tracking_rad']=max(worst['tracking_rad'],float(max(abs(d.qpos[:16]-d.ctrl))));worst['range_rad']=max(worst['range_rad'],float(max(0,max(nlo-d.qpos[:16]),max(d.qpos[:16]-nhi))));C=d.qpos[-7:-4];B=d.xmat[body].reshape(3,3);inhand=external=False;pressures={'if_tip':0.,'th_tip':0.}
            for ci,c in enumerate(d.contact):
                x,y=int(c.geom1),int(c.geom2)
                if cube not in (x,y):continue
                other=y if x==cube else x
                if int(m.geom_bodyid[other]) in hand:
                    inhand|=c.dist<.0004;worst['cube_mm']=max(worst['cube_mm'],max(0.,-float(c.dist)*1000));name=m.geom(other).name
                    if name in pressures and -float((c.pos-C)@B[:,1])*1000<55/6:
                        ff=np.zeros(6);mujoco.mj_contactForce(m,d,ci,ff);pressures[name]+=float(ff[0])
                elif other in support and c.dist<.0004:external=True
            lowergood+=all(v>.2 for v in pressures.values());gap=0. if inhand else gap+.001;longest=max(longest,gap);maxyaw=max(maxyaw,abs(angle(B,B0,axis)));maxshift=max(maxshift,float(np.linalg.norm(C-C0)*1000));maxtilt=max(maxtilt,float(np.degrees(np.arccos(np.clip(B[:,1]@B0[:,1],-1,1)))));maxpose=max(maxpose,orientation_error_deg(B,B0))
            if not cert_bad:
                clear,bad=cert.query(d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3));minclear=min(minclear,clear);cert_bad+=bool(bad)
            else:cert_skipped+=1
            if not np.isfinite(np.r_[d.qpos,d.qvel]).all() or worst['self_mm']>1e-6 or worst['support_mm']>.15 or worst['cube_mm']>1 or worst['tracking_rad']>.18 or worst['range_rad']>1e-6 or external or longest>.25 or np.linalg.norm(C-C0)>.08:stop='original_dynamic_bound';break
        if sample%13==0 or stop or sample==133:trace.append(dict(seconds=count*.001,yaw_change_deg=angle(B,B0,axis),shift_mm=float(np.linalg.norm(C-C0)*1000),pressures_N=pressures,certified_gap_mm=minclear))
        if stop:break
    why=[]
    if stop or count!=10050:why.append('original_dynamic_guard_or_incomplete')
    if maxyaw>=3 or maxtilt>=3 or maxpose>=3:why.append('body_orientation_changed')
    if maxshift>=2:why.append('body_translated')
    if cert_bad or cert_skipped or cert.queries!=count or minclear<=.3:why.append('upper_sweep_clearance')
    if lowergood/max(1,count)<=.8:why.append('lower_contacts')
    goals=np.asarray(rows);assert max(abs(np.diff(np.vstack([start,goals]),axis=0)).ravel())<=2;np.savez_compressed(out/'goals.npz',start=start,goals=goals);np.savez_compressed(out/'end_state.npz',qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,act=d.act,qacc_warmstart=d.qacc_warmstart,time=d.time,up=z['up'],initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'],required_lower_contacts=np.asarray(['if_tip','th_tip']));return dict(mode=mode,torque_Nm=torque,steps=count,passed=not why,reasons=why,guard_stop=stop,max_yaw_change_deg=maxyaw,max_tilt_change_deg=maxtilt,max_orientation_change_deg=maxpose,max_shift_mm=maxshift,upper_gap_all1ms_mm=minclear,lower_contact_fraction=lowergood/max(1,count),certificate=cert.metadata(),certificate_bad_steps=cert_bad,certificate_skipped_steps=cert_skipped,worst=worst,trace=trace,controller_never_reads_applied_load=True,targets_change_only_within2ticks=True,elapsed_seconds=time.perf_counter()-began)
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--mode',choices=('force','hybrid'),required=True);ap.add_argument('--profile',choices=tuple(PROFILES),default='baseline');a=ap.parse_args();source=ROOT/'continuous_handoff/end_state.npz';model=ROOT/'torque_cap_sensitivity/cap0.15/model.mjb';z=np.load(source);m=mujoco.MjModel.from_binary_path(str(model));out=ROOT/'feedback_body_hold'/(a.mode+('' if a.profile=='baseline' else '_'+a.profile));out.mkdir(parents=True,exist_ok=True);cases=[]
    for torque in (0.,-.01,.01):
        folder=out/f'torque{torque:+g}';folder.mkdir(exist_ok=True);v=trial(m,z,torque,a.mode,folder,a.profile);cases.append(v);report=dict(status='perfect_simulated_pose_feedback_hold_under_unknown_load',profile=a.profile,normal_reference_N=PROFILES[a.profile][0],kinematic_pose_gain_scale=PROFILES[a.profile][1],source_state=source.relative_to(R).as_posix(),source_state_sha256=sha(source),source_model_sha256=sha(model),cases=cases,not_a_live_camera_or_SAC_controller=True,controller_does_not_read_the_load=True,native_geometry_force_limits_and_health_bounds_unchanged=True,devices_opened=False,hardware_execution_allowed=False);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in v.items() if k not in ('trace','certificate')}),flush=True)
if __name__=='__main__':main()
