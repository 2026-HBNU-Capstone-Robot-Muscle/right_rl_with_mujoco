"""Rotate an ideal separate upper layer while right-hand targets stay fixed.

The layer is driven by an external simulated torque, not an internal motor:
its uncalibrated bearing transmits the load to the lower two thirds. This is
an isolated contact/load experiment, not the missing real left-hand model.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import inspect_for,native,angle
from upper_layer_separation_certificate_20261105 import SweepCertificate

BASE=ROOT/'articulated_upper_layer'

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--bearing',type=float,choices=(.001,.003,.01),required=True);ap.add_argument('--direction',type=int,choices=(-1,1),required=True);ap.add_argument('--friction-audit',action='store_true');a=ap.parse_args()
    model=BASE/'model.mjb';source=ROOT/'full_placement_v3_thumb12/front-10/g1_f0.3/end_state.npz';z=np.load(source)
    if 'top_face_id' in z and int(z['top_face_id'])!=5:raise ValueError('this separate layer model has GREEN5 as its working layer')
    m=mujoco.MjModel.from_binary_path(str(model));d=mujoco.MjData(m)
    assert m.nq==len(z['qpos'])+1 and m.nv==len(z['qvel'])+1
    d.qpos[:len(z['qpos'])]=z['qpos'];d.qvel[:len(z['qvel'])]=z['qvel'];d.ctrl[:]=z['ctrl'];d.act[:]=z['act'];d.time=float(z['time'])
    j=m.joint('upper_layer_axis').id;qa=int(m.jnt_qposadr[j]);va=int(m.jnt_dofadr[j]);m.dof_frictionloss[va]=a.bearing
    d.qpos[qa]=0;d.qvel[va]=0;mujoco.mj_forward(m,d)
    body=m.body('cube').id;upper=m.body('upper_layer').id;cubegeoms={m.geom('cube').id,m.geom('upper_layer_collision').id}
    cube_joint=next(k for k in range(m.njnt) if int(m.jnt_bodyid[k])==body and int(m.jnt_type[k])==mujoco.mjtJoint.mjJNT_FREE)
    cubeqa=int(m.jnt_qposadr[cube_joint]);C0=d.qpos[cubeqa:cubeqa+3].copy();B0=d.xmat[body].reshape(3,3).copy();up=z['up'];control=d.ctrl.copy()
    hand,support,check=inspect_for(m);_,_,nlo,nhi=native(m);cert=SweepCertificate(m,hand,.3)
    worst=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,range_rad=0.)
    source_properties={k:getattr(m,k).copy() for k in ('actuator_gainprm','actuator_biasprm','actuator_forcerange','jnt_range','geom_size','geom_contype','geom_conaffinity')}
    out=BASE/f'bearing{a.bearing:g}_direction{a.direction:+d}{"_friction_audit" if a.friction_audit else ""}';out.mkdir(parents=True,exist_ok=True)
    trace=[];snapshots=[];snapshot_times=[];maxyaw=maxshift=maxtilt=0.;minimum_gap=float('inf');certificate_bad=certificate_skipped=lower_steps=steps=0
    gap=longest=0.;stop=None;max_torque=0.;max_hinge_constraint=0.;upper_contact_steps=0;began=time.perf_counter()
    for step in range(10050):
        seconds=step*.001
        fraction=float(np.clip((seconds-1)/6,0,1));smooth=fraction*fraction*(3-2*fraction);desired=a.direction*np.pi/2*smooth
        desired_velocity=a.direction*np.pi/2*6*fraction*(1-fraction)/6 if 1<seconds<7 else 0.
        tau=float(np.clip(.3*(desired-d.qpos[qa])+.003*(desired_velocity-d.qvel[va]),-.02,.02))
        axis=d.xmat[body].reshape(3,3)@np.array([0.,-1,0]);d.xfrc_applied[upper,3:]=axis*tau;max_torque=max(max_torque,abs(tau))
        mujoco.mj_step(m,d);steps+=1;max_hinge_constraint=max(max_hinge_constraint,abs(float(d.qfrc_constraint[va])))
        su,sp=check(d);worst['self_mm']=max(worst['self_mm'],max(sp.values(),default=0.));worst['support_mm']=max(worst['support_mm'],max(su.values(),default=0.));worst['tracking_rad']=max(worst['tracking_rad'],float(max(abs(d.qpos[:16]-d.ctrl))));worst['range_rad']=max(worst['range_rad'],float(max(0,max(nlo-d.qpos[:16]),max(d.qpos[:16]-nhi))))
        C=d.qpos[cubeqa:cubeqa+3];B=d.xmat[body].reshape(3,3);inhand=external=False;contacts={};upper_touch=False
        for ci,c in enumerate(d.contact):
            x,y=int(c.geom1),int(c.geom2)
            if x not in cubegeoms and y not in cubegeoms:continue
            if x in cubegeoms and y in cubegeoms:continue
            other=y if x in cubegeoms else x;which=x if x in cubegeoms else y
            if int(m.geom_bodyid[other]) in hand:
                inhand|=c.dist<.0004;worst['cube_mm']=max(worst['cube_mm'],max(0.,-float(c.dist)*1000));force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force)
                height=-float((c.pos-C)@B[:,1])*1000
                if height<55/6:
                    name=m.geom(other).name;contacts[name]=contacts.get(name,0.)+float(force[0])
                if which==m.geom('upper_layer_collision').id and force[0]>.01:upper_touch=True
            elif other in support and c.dist<.0004:external=True
        upper_contact_steps+=upper_touch;lower_steps+=all(contacts.get(name,0.)>.2 for name in ('mf_tip','th_tip'))
        gap=0. if inhand else gap+.001;longest=max(longest,gap)
        maxyaw=max(maxyaw,abs(angle(B,B0,up)));maxshift=max(maxshift,float(np.linalg.norm(C-C0)*1000));maxtilt=max(maxtilt,float(np.degrees(np.arccos(np.clip(B[:,1]@B0[:,1],-1,1)))))
        if not certificate_bad:
            certified,bad=cert.query(d,C,B);minimum_gap=min(minimum_gap,certified);certificate_bad+=bool(bad)
        else:certificate_skipped+=1
        if not np.array_equal(control,d.ctrl):raise RuntimeError('right-hand targets changed')
        if (not np.isfinite(np.r_[d.qpos,d.qvel]).all() or worst['self_mm']>1e-6 or worst['support_mm']>.15 or worst['cube_mm']>1 or worst['tracking_rad']>.18 or worst['range_rad']>1e-6 or external or longest>.25 or np.linalg.norm(C-C0)>.08):stop='original hand/contact/range/tracking/retention bound';break
        if step%150==0 or step==10049:
            trace.append(dict(time_s=seconds,upper_angle_deg=float(np.degrees(d.qpos[qa])),upper_angular_rate_rad_s=float(d.qvel[va]),desired_angle_deg=float(np.degrees(desired)),external_layer_torque_nm=tau,hinge_generalized_constraint_torque_nm=float(d.qfrc_constraint[va]),lower_body_yaw_change_deg=angle(B,B0,up),lower_body_shift_mm=float(np.linalg.norm(C-C0)*1000),lower_contacts_N=contacts));snapshots.append(d.qpos.copy());snapshot_times.append(seconds)
    for k,value in source_properties.items():assert np.array_equal(getattr(m,k),value),k
    reasons=[]
    if stop or steps!=10050:reasons.append('original_dynamic_guard_or_incomplete')
    upper_error=abs(float(np.degrees(d.qpos[qa]))-a.direction*90)
    if upper_error>=3:reasons.append('upper_layer_not_at90')
    if maxyaw>=3:reasons.append('lower_body_yaw_changed')
    if maxshift>=2:reasons.append('lower_body_shifted')
    if maxtilt>=3:reasons.append('lower_body_tilt_changed')
    if certificate_bad or certificate_skipped or cert.queries!=steps or minimum_gap<=.3:reasons.append('upper_sweep_clearance')
    if upper_contact_steps:reasons.append('right_hand_contacted_upper_layer')
    if lower_steps/max(1,steps)<=.8:reasons.append('lower_body_contact_presence')
    np.savez_compressed(out/'trajectory.npz',qpos=snapshots,time_s=snapshot_times)
    report=dict(status='articulated_upper_layer_offline_load_hypothesis',source_state=str(source.relative_to(R)),source_state_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),model=str(model.relative_to(R)),model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),bearing_frictionloss_hypothesis_nm=a.bearing,direction=a.direction,steps=steps,upper_final_angle_deg=float(np.degrees(d.qpos[qa])),upper_target_error_deg=upper_error,maximum_external_layer_torque_nm=max_torque,maximum_lower_body_yaw_change_deg=maxyaw,maximum_lower_body_shift_mm=maxshift,maximum_lower_body_tilt_change_deg=maxtilt,minimum_upper_sweep_gap_1ms_mm=minimum_gap,certificate_bad_steps=certificate_bad,certificate_unqueried_steps=certificate_skipped,certificate=cert.metadata(),right_hand_upper_layer_contact_steps=upper_contact_steps,lower_contact_presence_fraction=lower_steps/max(1,steps),right_hand_targets_constant=True,original_hand_dynamic_safety_passed=stop is None and steps==10050,guard_stop=stop,load_filter_passed=not reasons,reasons=reasons,worst=worst,trace=trace,elapsed_seconds=time.perf_counter()-began,external_torque_not_an_internal_hinge_motor=True,layer_bearing_and_left_hand_load_not_physically_calibrated=True,actual_left_hand_geometry_absent=True,not_a_full_rubik_mechanism=True,full_placement_not_replayed_in_articulated_model=True,hardware_execution_allowed=False,devices_opened=False)
    report.update(maximum_absolute_hinge_constraint_torque_nm=max_hinge_constraint,model_disable_flags=int(m.opt.disableflags),frictionloss_disabled=bool(int(m.opt.disableflags)&int(mujoco.mjtDisableBit.mjDSBL_FRICTIONLOSS)),bearing_parameter_is_not_a_measurement_of_transmitted_torque=True)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k not in ('trace','certificate')}),flush=True)
if __name__=='__main__':main()
