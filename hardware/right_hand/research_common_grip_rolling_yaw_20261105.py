"""Free-cube simulated feedback with migrating fingertip contact points.

This isolated offline optimizer retains fractional tick decisions. It has no
camera or motor adapter and is not a live SAC policy. A separate command-pose
probe estimates contacts; task angles come only from the freely simulated cube.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from scipy.optimize import lsq_linear
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor,angle
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK,ticks_to_sim
from convex_distance_check_20261104 import CheckedDistance
from upper_layer_separation_certificate_20261105 import SweepCertificate
from offline_fractional_goal_accumulator_20261105 import FractionalGoals

A=np.r_[np.arange(4,8),np.arange(12,16)]
IDS=np.array(SIM_TO_MOTOR_IDS)-1

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--mode',choices=('rolling','tangential'),default='rolling');ap.add_argument('--depth-scale',type=float,choices=(.8,1.,1.2,1.5),default=1.);ap.add_argument('--integer-restart',action='store_true');ap.add_argument('--palm-clearance-mm',type=float,choices=(0.,.5),default=0.);ap.add_argument('--level',action='store_true');ap.add_argument('--center-feedback',action='store_true');ap.add_argument('--lift-mm',type=float,choices=(0.,8.),default=0.);ap.add_argument('--samples',type=int,default=900)
    ap.add_argument('--source-choice',choices=('common','workspace_xpos'),default='common');a=ap.parse_args()
    if a.lift_mm and not a.center_feedback:raise ValueError('lift requires centre return feedback')
    if not 200<=a.samples<=1400:raise ValueError('bounded offline workload')
    tag=f'{a.mode}_depth{a.depth_scale:g}_{"integer_restart" if a.integer_restart else "fractional"}'+('_palmclear' if a.palm_clearance_mm else '')+('_level' if a.level else '')+('_center' if a.center_feedback else '')+(f'_lift{a.lift_mm:g}' if a.lift_mm else '')+('_workspace_xpos' if a.source_choice!='common' else '');out=ROOT/'rolling_feedback'/tag;out.mkdir(parents=True,exist_ok=True)
    folder=ROOT/'full_placement_v3_thumb12/front-10/g1_f0.3'
    source=folder/'end_state.npz';model=folder/'model.mjb'
    if a.source_choice=='workspace_xpos':
        from dynamic_common_grip_search_20261105 import CP
        source=ROOT/'workspace_transition/xpos_forward/force1.5/end_state.npz';model=CP/'end_model.mjb'
    m=mujoco.MjModel.from_binary_path(str(model));z=np.load(source);d=restore(m,z);p=mujoco.MjData(m);cube=m.geom('cube').id;body=m.body('cube').id;hand,support,check=inspect_for(m);lo,hi,nlo,nhi=native(m);up=z['up'];Cstart=d.qpos[-7:-4].copy();Binit=z['initial_cube_rotation'];start=motor(d.ctrl);planner=FractionalGoals(start,np.ceil(lo)+4,np.floor(hi)-4);tips=[m.geom(name).id for name in ('mf_tip','th_tip')];checked=CheckedDistance(m,cube,tips);cert=SweepCertificate(m,hand,.3);unchanged=[m.actuator_forcerange.copy(),m.actuator_gainprm.copy(),m.actuator_biasprm.copy()]
    p.qpos[:]=d.qpos;p.qpos[:16]=ticks_to_sim(planner.continuous);p.ctrl[:]=p.qpos[:16];p.qvel[:]=0;mujoco.mj_forward(m,p)
    initial_distances=np.array([checked.distance(p,g)[0] for g in tips]);target_distances=initial_distances.copy();target_distances[initial_distances<0]*=a.depth_scale
    barrier_probe=mujoco.MjData(m);palm_boxes=[g for g in range(m.ngeom) if (m.geom(g).name or '').startswith('palm_collision_') and int(m.geom_type[g])==6];barrier_checks={g:CheckedDistance(m,g,tips) for g in palm_boxes};barrier_uses=0
    rows=[];trace=[];snapshots=[];snapshot_index=[];goal_ready=False;lowering=False;hold=0;stop=None;search_stop=None;gap=longest=0.;yaw=angle(d.xmat[body].reshape(3,3),Binit,up);worst=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,range_rad=0.);minimum_gap=float('inf');certificate_bad=certificate_skipped=lower_steps=tail_steps=0;hold_B=hold_C=None;hold_yaw_drift=hold_shift=0.;began=time.perf_counter()
    for index in range(1,a.samples+1):
        if not goal_ready:
            if a.integer_restart:planner.continuous=planner.goal.astype(float)
            p.qpos[:]=d.qpos;p.qpos[:16]=ticks_to_sim(planner.continuous);p.ctrl[:]=p.qpos[:16];p.qvel[:]=0;mujoco.mj_forward(m,p);normal_j=[];normal_error=[];tangent_j=[];tangent_goal=[];rotation_j=[];rotation_goal=[];axis=-d.xmat[body].reshape(3,3)[:,1];increment=float(np.clip(np.radians(90-yaw)*.2,-.0025,.0025));omega=axis*increment;velocity=np.zeros(3)
            if a.level:omega=up*increment+np.cross(axis,up)*.05
            if a.center_feedback:
                target_center=Cstart+up*(0. if lowering else a.lift_mm/1000);velocity=(target_center-d.qpos[-7:-4])*.05;length=np.linalg.norm(velocity)
                if length>.00015:velocity*=.00015/length
            for role,g in enumerate(tips):
                distance,pair=checked.distance(p,g);contacts=[c for c in p.contact if {int(c.geom1),int(c.geom2)}=={cube,g}]
                if contacts:
                    c=min(contacts,key=lambda c:c.dist);normal=c.frame[:3]*(1 if int(c.geom1)==cube else -1);point=c.pos.copy()
                elif abs(distance)>1e-9:normal=(pair[3:]-pair[:3])/distance;normal/=np.linalg.norm(normal);point=pair[3:]
                else:search_stop='undefined_command_pose_contact_normal';break
                jp=np.zeros((3,m.nv));jr=np.zeros((3,m.nv));mujoco.mj_jac(m,p,jp,jr,point,int(m.geom_bodyid[g]));project=np.eye(3)-np.outer(normal,normal);normal_j.append(normal@jp[:,A]);normal_error.append(target_distances[role]-distance+(normal@velocity if index>30 else 0.));tangent_j.append(project@jp[:,A]);tangent_goal.append(project@(velocity+np.cross(omega,point-d.qpos[-7:-4])) if index>30 else np.zeros(3));rotation_j.append(np.column_stack([np.cross(jr[:,j],normal) for j in A]));rotation_goal.append(np.cross(omega,normal) if index>30 else np.zeros(3))
            if search_stop:break
            matrix=np.vstack([np.array(normal_j)*10000,np.vstack(tangent_j)*2000,np.eye(len(A))*.02]);rhs=np.r_[np.array(normal_error)*10000,np.concatenate(tangent_goal)*2000,np.zeros(len(A))]
            if a.mode=='rolling':matrix=np.vstack([matrix,np.vstack(rotation_j)*20]);rhs=np.r_[rhs,np.concatenate(rotation_goal)*20]
            if a.palm_clearance_mm:
                barrier_probe.qpos[:]=d.qpos;barrier_probe.qpos[:16]=ticks_to_sim(planner.continuous)+(d.qpos[:16]-d.ctrl);barrier_probe.qvel[:]=0;mujoco.mj_forward(m,barrier_probe)
                for pg,checker in barrier_checks.items():
                    for tip in tips:
                        near=float(mujoco.mj_geomDistance(m,barrier_probe,pg,tip,.003,None))
                        if near>=.003:continue
                        distance,pair=checker.distance(barrier_probe,tip)
                        if distance>=a.palm_clearance_mm/1000:continue
                        contacts=[c for c in barrier_probe.contact if {int(c.geom1),int(c.geom2)}=={pg,tip}]
                        if contacts:normal=contacts[0].frame[:3]*(1 if int(contacts[0].geom1)==pg else -1)
                        elif abs(distance)>1e-9:normal=(pair[3:]-pair[:3])/distance;normal/=np.linalg.norm(normal)
                        else:normal=barrier_probe.geom_xpos[tip]-barrier_probe.geom_xpos[pg];normal/=np.linalg.norm(normal)
                        ja=np.zeros((3,m.nv));jb=ja.copy();mujoco.mj_jac(m,barrier_probe,ja,None,pair[:3],int(m.geom_bodyid[pg]));mujoco.mj_jac(m,barrier_probe,jb,None,pair[3:],int(m.geom_bodyid[tip]));row=normal@(jb[:,A]-ja[:,A]);matrix=np.vstack([matrix,row*20000]);rhs=np.r_[rhs,(a.palm_clearance_mm/1000-distance)*20000];barrier_uses+=1
            low,high=planner.increment_bounds();lower=low[IDS[A]]*TICK;upper=high[IDS[A]]*TICK;fixed=upper-lower<1e-12;change=np.zeros(len(A));change[fixed]=(lower[fixed]+upper[fixed])/2;free=~fixed
            if np.any(free):change[free]=lsq_linear(matrix[:,free],rhs-matrix[:,fixed]@change[fixed],bounds=(lower[free],upper[free]),tol=1e-10,max_iter=100).x
            delta=np.zeros(16);delta[IDS[A]]=change/TICK;goal=planner.advance(delta)
        else:hold+=1;goal=planner.goal.copy()
        d.ctrl[:]=ticks_to_sim(goal);rows.append(goal.copy())
        for sub in range(75):
            mujoco.mj_step(m,d);su,sp=check(d);worst['self_mm']=max(worst['self_mm'],max(sp.values(),default=0.));worst['support_mm']=max(worst['support_mm'],max(su.values(),default=0.));worst['tracking_rad']=max(worst['tracking_rad'],float(max(abs(d.qpos[:16]-d.ctrl))));worst['range_rad']=max(worst['range_rad'],float(max(0,max(nlo-d.qpos[:16]),max(d.qpos[:16]-nhi))));contacts={};lower_contacts={};inhand=external=False;B=d.xmat[body].reshape(3,3);C=d.qpos[-7:-4]
            for ci,c in enumerate(d.contact):
                x,y=int(c.geom1),int(c.geom2)
                if cube not in (x,y):continue
                other=y if x==cube else x
                if int(m.geom_bodyid[other]) in hand:
                    inhand|=c.dist<.0004;worst['cube_mm']=max(worst['cube_mm'],max(0.,-float(c.dist)*1000));force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force);name=m.geom(other).name;contacts[name]=contacts.get(name,0.)+float(force[0]);height=-float((c.pos-C)@B[:,1])*1000
                    if height<55/6:lower_contacts[name]=lower_contacts.get(name,0.)+float(force[0])
                elif other in support and c.dist<.0004:external=True
            gap=0. if inhand else gap+.001;longest=max(longest,gap)
            if goal_ready:
                if hold_B is None:hold_B=B.copy();hold_C=C.copy()
                hold_yaw_drift=max(hold_yaw_drift,abs(angle(B,hold_B,up)));hold_shift=max(hold_shift,float(np.linalg.norm(C-hold_C)*1000))
            if hold>=81:
                tail_steps+=1;lower_steps+=all(lower_contacts.get(name,0.)>.2 for name in ('mf_tip','th_tip'))
                if not certificate_bad:
                    clearance,bad=cert.query(d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3));minimum_gap=min(minimum_gap,clearance);certificate_bad+=bool(bad)
                else:certificate_skipped+=1
            if (not np.isfinite(np.r_[d.qpos,d.qvel,d.ctrl]).all() or worst['self_mm']>1e-6 or worst['support_mm']>.15 or worst['cube_mm']>1 or worst['tracking_rad']>.18 or worst['range_rad']>1e-6 or external or longest>.25 or np.linalg.norm(C-Cstart)>.08):stop='unchanged contact/range/tracking/retention bound';break
        B=d.xmat[body].reshape(3,3);yaw=angle(B,Binit,up);tilt=float(np.degrees(np.arccos(np.clip((-B[:,1])@up,-1,1))))
        if a.lift_mm and yaw>=89.25:lowering=True
        returned=not a.center_feedback or np.linalg.norm(d.qpos[-7:-4]-Cstart)<.001
        if not goal_ready and abs(yaw-90)<.75 and tilt<5 and returned:goal_ready=True
        if index%2==1 or stop or hold==134:snapshots.append(d.qpos.copy());snapshot_index.append(index)
        if index%40==0 or stop or hold==134:trace.append(dict(sample=index,yaw_deg=yaw,up_tilt_deg=tilt,goal=goal.tolist(),contact_N=contacts));print(json.dumps(dict(tag=tag,sample=index,yaw=yaw,tilt=tilt,holding=goal_ready,stop=stop)),flush=True)
        if stop or hold>=134:break
        if index>200 and index%100==0 and len(rows)>=60 and np.all(np.array(rows[-60:])==rows[-1]):search_stop='quantized_goals_stationary_for60_cycles';break
    completed_hold=goal_ready and hold>=134 and stop is None;direction=bool(completed_hold and abs(yaw-90)<5 and tilt<5 and hold_yaw_drift<3 and hold_shift<2);common=bool(direction and tail_steps==4050 and not certificate_bad and not certificate_skipped and minimum_gap>.3 and lower_steps/max(1,tail_steps)>.8 and np.linalg.norm(d.qpos[-7:-4]-Cstart)*1000<2)
    for value,reference in zip((m.actuator_forcerange,m.actuator_gainprm,m.actuator_biasprm),unchanged):assert np.array_equal(value,reference)
    np.savez_compressed(out/'suffix.npz',start=start,goals=rows);np.savez_compressed(out/'trajectory.npz',samples=snapshot_index,qpos=snapshots,goal_period_s=.075);np.savez_compressed(out/'end_state.npz',qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,act=d.act,qacc_warmstart=d.qacc_warmstart,time=d.time,up=up,initial_cube_rotation=Binit,initial_cube_center=z['initial_cube_center'])
    report=dict(status='free_cube_simulated_feedback_from_common_grip',mode=a.mode,depth_scale=a.depth_scale,fractional_planner_state_retained=not a.integer_restart,planned_extra_palm_clearance_mm=a.palm_clearance_mm,palm_barrier_uses=barrier_uses,up_axis_feedback_enabled=a.level,center_feedback_enabled=a.center_feedback,temporary_target_lift_mm=a.lift_mm,source_state=str(source.relative_to(R)),source_state_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),samples=len(rows),safety_passed=stop is None,guard_stop=stop,search_stop=search_stop,final_yaw_deg=yaw,final_up_tilt_deg=tilt,hold_samples=hold,hold_yaw_drift_deg=hold_yaw_drift,hold_translation_mm=hold_shift,whole90_direction_filter_passed=direction,common90_filter_passed=common,minimum_layer_gap_mm=minimum_gap if np.isfinite(minimum_gap) else None,certificate_bad_steps=certificate_bad,certificate_unqueried_steps=certificate_skipped,certificate=cert.metadata(),source_virtual_distances_mm=(initial_distances*1000).tolist(),target_virtual_distances_mm=(target_distances*1000).tolist(),final_center_shift_from_common_mm=float(np.linalg.norm(d.qpos[-7:-4]-Cstart)*1000),opposed_lower_contacts_fraction=lower_steps/max(1,tail_steps),worst=worst,trace=trace,elapsed_seconds=time.perf_counter()-began,all_original_dynamic_bounds_unchanged=True,actuator_properties_unchanged=True,perfect_simulated_object_state_feedback=True,full_initial_placement_not_replayed=True,cube_pose_never_reset_after_start=True,live_SAC=False,hardware_execution_allowed=False,devices_opened=False)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k not in ('trace','certificate')}),flush=True)
if __name__=='__main__':main()
