"""Continuous loaded replay with unchanged dynamic guards and hold metrics."""
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import restore,inspect_for,native,angle
from hand_motor_mapping import ticks_to_sim
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from upper_layer_separation_certificate_20261105 import SweepCertificate
from cube_task_frame_20261105 import normal,local_working_basis

def lower_contact_roles(required_lower_contacts):
    required_lower_contacts=tuple(required_lower_contacts)
    if not 2<=len(required_lower_contacts)<=4 or len(set(required_lower_contacts))!=len(required_lower_contacts) or not set(required_lower_contacts)<= {'if_tip','mf_tip','rf_tip','th_tip'}:raise ValueError('explicit distinct lower finger contacts required')
    return required_lower_contacts

def run(m,z,rows,forward_count,save=None,record=None,certify_tail=True,top_face_id=5,required_lower_contacts=('mf_tip','th_tip')):
    required_lower_contacts=lower_contact_roles(required_lower_contacts)
    d=restore(m,z);cube=m.geom('cube').id;body=m.body('cube').id;hand,support,check=inspect_for(m);lo,hi,nlo,nhi=native(m);rows=np.asarray(rows,int)
    if np.any(rows<lo) or np.any(rows>hi):raise ValueError('native goal bound')
    C0=d.qpos[-7:-4].copy();up=z['up'];B0=d.xmat[body].reshape(3,3).copy();top_normal=normal(top_face_id);work_basis=local_working_basis(top_face_id);cache=precompute(m,hand);cert=SweepCertificate(m,hand,.3) if certify_tail else None
    worst=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,range_rad=0.);stop=None;gap=longest=0.;completed=0;forward_B=None;forward_C=None;whole_hold_B=whole_hold_C=None;tail_B=tail_C=None;whole_drift=whole_shift=tail_drift=tail_shift=0.;whole_gap=tail_gap=float('inf');certificate_gap=float('inf');certificate_bad_steps=0;certificate_skipped_steps=0;lower_good=tail_steps=0;trace=[];snapshots=[];snapshot_q=[]
    for index,goal in enumerate(rows):
        d.ctrl[:]=ticks_to_sim(goal)
        for sub in range(75):
            mujoco.mj_step(m,d);su,sp=check(d);worst['self_mm']=max(worst['self_mm'],max(sp.values(),default=0.));worst['support_mm']=max(worst['support_mm'],max(su.values(),default=0.));worst['tracking_rad']=max(worst['tracking_rad'],float(max(abs(d.qpos[:16]-d.ctrl))));worst['range_rad']=max(worst['range_rad'],float(max(0,max(nlo-d.qpos[:16]),max(d.qpos[:16]-nhi))))
            B=d.xmat[body].reshape(3,3);C=d.qpos[-7:-4];inhand=external=False;body_contacts={};contacts={}
            for ci,c in enumerate(d.contact):
                a,b=int(c.geom1),int(c.geom2)
                if cube not in (a,b):continue
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    inhand|=c.dist<.0004;worst['cube_mm']=max(worst['cube_mm'],max(0.,-float(c.dist)*1000))
                    if index>=len(rows)-134 or index==forward_count-1:
                        ff=np.zeros(6);mujoco.mj_contactForce(m,d,ci,ff);name=m.geom(other).name;contacts[name]=contacts.get(name,0.)+float(ff[0]);height=-float((c.pos-C)@B[:,1])*1000 if top_face_id==5 else float((c.pos-C)@(B@top_normal))*1000
                        if height<55/6:body_contacts[name]=body_contacts.get(name,0.)+float(ff[0])
                elif other in support and c.dist<.0004:external=True
            gap=0. if inhand else gap+.001;longest=max(longest,gap)
            if index>=len(rows)-134:
                if whole_hold_B is None:whole_hold_B=B.copy();whole_hold_C=C.copy()
                whole_drift=max(whole_drift,abs(angle(B,whole_hold_B,up)));whole_shift=max(whole_shift,float(np.linalg.norm(C-whole_hold_C)*1000))
            if index>=len(rows)-54:
                if tail_B is None:tail_B=B.copy();tail_C=C.copy()
                tail_steps+=1;tail_drift=max(tail_drift,abs(angle(B,tail_B,up)));tail_shift=max(tail_shift,float(np.linalg.norm(C-tail_C)*1000));lower_good+=all(body_contacts.get(n,0.)>.2 for n in required_lower_contacts)
                if cert is not None and certificate_bad_steps==0:
                    box_rotation=d.geom_xmat[cube].reshape(3,3);certified,bad=cert.query(d,d.geom_xpos[cube],box_rotation if top_face_id==5 else box_rotation@work_basis);certificate_gap=min(certificate_gap,certified);certificate_bad_steps+=bool(bad)
                elif cert is not None:certificate_skipped_steps+=1
            if worst['self_mm']>1e-6 or worst['support_mm']>.15 or worst['cube_mm']>1 or worst['tracking_rad']>.18 or worst['range_rad']>1e-6 or external or longest>.25 or np.linalg.norm(C-C0)>.08:
                stop='unchanged contact/range/tracking/retention bound';break
        completed=index+1
        if completed==forward_count:forward_B=B.copy();forward_C=C.copy()
        if index>=len(rows)-134:
            box_rotation=d.geom_xmat[cube].reshape(3,3);cl=gaps(m,d,d.geom_xpos[cube],box_rotation if top_face_id==5 else box_rotation@work_basis,cache,True);current_gap=min(v for name,v in cl);whole_gap=min(whole_gap,current_gap)
            if index>=len(rows)-54:tail_gap=min(tail_gap,current_gap)
        else:current_gap=None
        if index%20==0 or completed in (forward_count,len(rows)) or stop:trace.append(dict(sample=completed,whole_yaw_deg=angle(B,z['initial_cube_rotation'],up),relative_forward_yaw_deg=None if forward_B is None else angle(B,forward_B,up),upper_gap_mm=current_gap))
        if record is not None and (index%2==0 or completed in (forward_count,len(rows))):snapshots.append(completed);snapshot_q.append(d.qpos.copy())
        if stop:break
    final_B=d.xmat[body].reshape(3,3);final_C=d.qpos[-7:-4].copy();yaw=None if forward_B is None else angle(final_B,forward_B,up);selected_up=-final_B[:,1] if top_face_id==5 else final_B@top_normal;tilt=float(np.degrees(np.arccos(np.clip(selected_up@up,-1,1))))
    if save:np.savez_compressed(save,qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,act=d.act,qacc_warmstart=d.qacc_warmstart,time=d.time,up=up,initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'],top_face_id=np.array(top_face_id),required_lower_contacts=np.asarray(required_lower_contacts))
    if record is not None:np.savez_compressed(record,samples=snapshots,qpos=snapshot_q,goal_period_s=.075,top_face_id=np.array(top_face_id))
    tail=dict(seconds=tail_steps*.001,yaw_drift_deg=tail_drift,center_shift_mm=tail_shift,minimum_upper_gap_sampled_75ms_mm=tail_gap,opposed_lower_body_contact_fraction=lower_good/max(1,tail_steps),minimum_separating_plane_gap_1ms_mm=certificate_gap if cert else None,separation_certificate_bad_steps_1ms=certificate_bad_steps if cert else None,certificate_unqueried_steps_after_first_failure=certificate_skipped_steps,certificate_failure_is_latched_and_cannot_pass=True,certificate=cert.metadata() if cert else None)
    task=bool(stop is None and yaw is not None and abs(yaw)<5 and tilt<5 and tail_gap>.3 and tail_drift<3 and tail_shift<2 and lower_good/max(1,tail_steps)>.8 and (cert is None or certificate_bad_steps==0))
    return dict(top_face_id=int(top_face_id),top_face_explicitly_selected=True,required_lower_contacts=list(required_lower_contacts),lower_contact_presence_does_not_prove_antipodal_force_closure=True,safety_passed=stop is None,stop=stop,completed_goals=completed,total_goals=len(rows),worst=worst,forward_yaw_from_placement_deg=None if forward_B is None else angle(forward_B,z['initial_cube_rotation'],up),final_yaw_from_placement_deg=angle(final_B,z['initial_cube_rotation'],up),regrasp_yaw_change_deg=yaw,final_up_tilt_deg=tilt,final_center_world_m=final_C.tolist(),final_rotation_world=final_B.tolist(),regrasp_translation_mm=None if forward_C is None else float(np.linalg.norm(final_C-forward_C)*1000),full_fixed_hold=dict(seconds=10.05 if completed==len(rows) else None,yaw_drift_deg=whole_drift,center_shift_mm=whole_shift,minimum_upper_gap_sampled_75ms_mm=whole_gap),settled_tail=tail,final_contact_N=contacts,whole_grip_task_passed=task,trace=trace)
