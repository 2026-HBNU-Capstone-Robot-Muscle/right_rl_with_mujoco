"""Level and finish90 after an opposing index/thumb grasp, isolated simulation.

Position goals only: the cube is never reset or externally forced. Uses exact
simulated contacts/pose, native pyramidal friction and the saved .15Nm model.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor,angle
from research_quasistatic_wrench_path_20261105 import allocate_native,hybrid_increment
from offline_fractional_goal_accumulator_20261105 import FractionalGoals
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg
from canonical_grip_dynamics_20261105 import run
from checkpoint_quarter_review_20261106 import quarter_reasons
from upper_layer_cylinder_clearance_20261105 import precompute,gaps

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--seed',type=int,choices=(0,1),default=0)
    ap.add_argument('--source-force',type=float,choices=(.5,1.),default=.5)
    ap.add_argument('--lift-mm',type=float,choices=(0.,2.,4.),default=2.)
    ap.add_argument('--normal-N',type=float,choices=(.3,1.,1.5,2.5),default=1.5)
    ap.add_argument('--samples',type=int,default=650)
    for flag in ('resume-leveled','lower-contacts','staged-slide','release-index-base','resume-freebase','target-station','via-lift','local-station','lock-contact-normals','recover-tip-contacts'):
        ap.add_argument('--'+flag,action='store_true')
    ap.add_argument('--settle-deg',type=float,choices=(1.5,3.),default=1.5)
    a=ap.parse_args()
    if a.staged_slide and not(a.resume_leveled and a.lower_contacts):raise ValueError('staged slide starts from the saved leveled grasp')
    if a.release_index_base and not a.staged_slide:raise ValueError('base release requires staged slide')
    if a.resume_freebase and (a.resume_leveled or a.staged_slide or a.lower_contacts):raise ValueError('free-base continuation uses its own saved source')
    if a.via_lift and not a.target_station:raise ValueError('via-lift is a station-transfer proposal')
    if a.local_station and (not a.resume_freebase or a.target_station):raise ValueError('local station continues the saved lower two-finger grip')
    if not 134<=a.samples<=900:raise ValueError('bounded offline search')
    parent=ROOT/'near_quarter_full_replay';r=json.loads((parent/'review.json').read_bytes());model=R/r['source_model'];assert sha(model)==r['source_model_sha256'];m=mujoco.MjModel.from_binary_path(str(model));source=ROOT/f'index_countergrip_workspace/p12_seeds6/zneg_{a.seed}/force{a.source_force:g}_end.npz'
    if a.resume_leveled:source=ROOT/'countergrip_level/seed0_source1_lift2_normal1.5_n850/verified_end_state.npz'
    if a.resume_freebase:source=ROOT/'countergrip_level/seed0_source1_lift0_normal2.5_n900_resume_lower_staged_freebase/end_state.npz'
    z=np.load(source);d=restore(m,z);hand,support,check=inspect_for(m);geometry=precompute(m,hand);cube=m.geom('cube').id;body=m.body('cube').id;up=z['up'];C0=d.qpos[-7:-4].copy();B0=d.xmat[body].reshape(3,3).copy();Ctarget=C0+up*a.lift_mm/1000;Btarget=upright_yaw_target(z['initial_cube_rotation'],up,90);rotvec=Rotation.from_matrix(Btarget@B0.T).as_rotvec();lo,hi,nlo,nhi=native(m);start=motor(d.ctrl);goal=start.copy();planner=FractionalGoals(start,np.minimum(lo+8,start),np.maximum(hi-8,start));reference={'if_tip':a.normal_N,'th_tip':a.normal_N};rows=[];trace=[];snapshots=[];stable=held=0;phase='level';stop=None;search_stop=None;gap=longest=0.;began=time.perf_counter();worst=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,range_rad=0.);height_targets={'if_tip':-.007,'th_tip':-.014} if a.lower_contacts else None;upper_gap=None
    station_source=None
    if a.target_station:
        station_source=ROOT/'postturn_regrasp_refined/middle_zneg/static_proposal.npz';Ctarget=np.load(station_source)['cube_center'].copy()
    if a.local_station:
        station_source=ROOT/'lower_countergrip_station/seed1/static_proposal.npz';Ctarget=np.load(station_source)['cube_center'].copy();height_targets={'if_tip':-.014,'th_tip':-.014}
    motion_seconds=45. if a.via_lift else (20. if a.target_station else 5.)
    if a.local_station:motion_seconds=20.
    if a.via_lift:
        travel_height=max(float(C0@up),float(Ctarget@up))+.006;raised_start=C0+up*(travel_height-float(C0@up));raised_end=Ctarget+up*(travel_height-float(Ctarget@up))
    tag=f'seed{a.seed}_source{a.source_force:g}_lift{a.lift_mm:g}_normal{a.normal_N:g}'+(f'_n{a.samples}' if a.samples!=650 else '')
    tag+=('_resume' if a.resume_leveled else '')+('_lower' if a.lower_contacts else '')
    tag+='_staged' if a.staged_slide else ''
    tag+='_freebase' if a.release_index_base else ''
    tag+=('_finish_freebase' if a.resume_freebase else '')+(f'_tol{a.settle_deg:g}' if a.settle_deg!=1.5 else '')
    tag+='_station_v2' if a.target_station else ''
    tag+='_via' if a.via_lift else ''
    tag+='_local_station' if a.local_station else ''
    tag+='_normal_lock' if a.lock_contact_normals else ''
    tag+='_tip_recovery' if a.recover_tip_contacts else ''
    out=ROOT/'countergrip_level'/tag;out.mkdir(parents=True,exist_ok=True)
    properties={k:getattr(m,k).copy() for k in ('actuator_gainprm','actuator_biasprm','actuator_forcelimited','actuator_forcerange','jnt_range','geom_size','geom_contype','geom_conaffinity')}
    if a.staged_slide:phase='unload'
    normal_frames=None
    if a.lock_contact_normals:
        normal_frames={}
        for name in ('if_tip','th_tip'):
            g=m.geom(name).id;contacts=[c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,g}]
            if contacts:
                c=min(contacts,key=lambda c:c.dist);normal=c.frame[:3]*(1 if int(c.geom1)==cube else -1)
            else:
                # A task direction is an intent, not an invented force reading.
                # Post-integration forward recomputation may have no contact at
                # this exact millisecond despite a high saved tail fraction.
                normal=B0[:,2]*(-1 if name=='if_tip' else 1)
            normal_frames[name]=(d.geom_xmat[g].reshape(3,3).T@normal,B0.T@normal)
    tip_memory=None
    if a.recover_tip_contacts:
        tip_memory={}
        for name in ('if_tip','th_tip'):
            g=m.geom(name).id;line=np.zeros(6);distance=float(mujoco.mj_geomDistance(m,d,g,cube,.1,line));contacts=[c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,g}]
            if contacts:
                c=min(contacts,key=lambda c:c.dist);point=c.pos.copy();target=point;normal=c.frame[:3]*(1 if int(c.geom1)==cube else -1)
            else:
                if distance>.003:raise ValueError('tip recovery source farther than3mm from the cube')
                point=line[:3].copy();target=line[3:].copy();normal=B0[:,2]*(-1 if name=='if_tip' else 1)
            tip_memory[name]=(d.geom_xmat[g].reshape(3,3).T@(point-d.geom_xpos[g]),B0.T@(target-C0),B0.T@normal)
    role_geometry={role:[v for v in geometry if m.body(int(m.geom_bodyid[v[0]])).name.startswith(prefix)] for role,prefix in (('if_tip','if_'),('th_tip','th_'))};role_gaps={};pressures={};stage_since=0;pressure_history=[];average_pressure={};held_thumb_height=-.014
    for index in range(1,a.samples+1):
        C=d.qpos[-7:-4].copy();B=d.xmat[body].reshape(3,3);blend=min(1,(index-1)*.075/motion_seconds);blend=blend*blend*(3-2*blend);Ct=C0+(Ctarget-C0)*blend;Bt=Rotation.from_rotvec(rotvec*blend).as_matrix()@B0
        if a.via_lift:
            segment=min(2,int((index-1)*.075/15));fraction=float(np.clip((index-1)*.075/15-segment,0,1));fraction=fraction*fraction*(3-2*fraction);pa,pb=((C0,raised_start),(raised_start,raised_end),(raised_end,Ctarget))[segment];Ct=pa+(pb-pa)*fraction
        if phase=='hold':held+=1
        else:
            if a.staged_slide:
                reference.update({role:(.3 if phase in ('unload','slide_th','slide_if') else a.normal_N) for role in ('if_tip','th_tip')})
                height_targets=None if phase=='unload' else ({'th_tip':-.014} if phase=='slide_th' else {'if_tip':-.007,'th_tip':-.014})
                if a.release_index_base:
                    if phase in ('preload','slide_if','tighten'):reference.update({role:a.normal_N for role in ('if_tip','th_tip')})
                    if phase=='preload':height_targets={'th_tip':held_thumb_height}
                    if phase in ('slide_if','tighten'):height_targets={'if_tip':held_thumb_height,'th_tip':held_thumb_height}
            velocity=np.zeros(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,body,velocity,0);F=-m.body_mass[body]*m.opt.gravity+30*(Ct-C)-1.5*velocity[3:];T=.04*Rotation.from_matrix(Bt@B.T).as_rotvec()-.002*velocity[:3];proposal,allocation=allocate_native(m,d,cube,C,F,T,reference,hand,True,(nlo,nhi),('if_tip','th_tip'),normal_reference_weight=2. if a.release_index_base or a.resume_freebase or a.via_lift else .2)
            if proposal is None:search_stop=allocation['reason'];break
            ignore=tuple(m.geom(v[0]).name for v in role_geometry['if_tip'] if m.geom(v[0]).name!='if_tip') if a.resume_freebase or (a.release_index_base and phase in ('slide_if','tighten')) else ()
            goal=hybrid_increment(m,d,allocation,planner,C,Ct,B,Bt,tip_memory=tip_memory,contact_height_targets=height_targets,world_up=up,maximum_tracking_rad=.12 if a.staged_slide or a.local_station or a.lock_contact_normals else None,ignored_motion_geoms=ignore,contact_normal_frames=normal_frames,tip_memory_names=('if_tip','th_tip'),recovery_normal_targets=reference)
        d.ctrl[:]=__import__('hand_motor_mapping').ticks_to_sim(goal);rows.append(goal.copy())
        for sub in range(75):
            mujoco.mj_step(m,d);su,sp=check(d);worst['self_mm']=max(worst['self_mm'],max(sp.values(),default=0.));worst['support_mm']=max(worst['support_mm'],max(su.values(),default=0.));worst['tracking_rad']=max(worst['tracking_rad'],float(max(abs(d.qpos[:16]-d.ctrl))));worst['range_rad']=max(worst['range_rad'],float(max(0,max(nlo-d.qpos[:16]),max(d.qpos[:16]-nhi))));inhand=external=False
            for c in d.contact:
                x,y=int(c.geom1),int(c.geom2)
                if cube not in (x,y):continue
                other=y if x==cube else x
                if int(m.geom_bodyid[other]) in hand:inhand|=c.dist<.0004;worst['cube_mm']=max(worst['cube_mm'],max(0.,-float(c.dist)*1000))
                elif other in support and c.dist<.0004:external=True
            gap=0. if inhand else gap+.001;longest=max(longest,gap)
            if (not np.isfinite(np.r_[d.qpos,d.qvel]).all() or worst['self_mm']>1e-6 or worst['support_mm']>.15 or worst['cube_mm']>1 or worst['tracking_rad']>.18 or worst['range_rad']>1e-6 or external or longest>.25 or np.linalg.norm(d.qpos[-7:-4]-C0)>.08):stop='original_dynamic_bound';break
        C=d.qpos[-7:-4].copy();B=d.xmat[body].reshape(3,3);position=float(np.linalg.norm(C-Ctarget)*1000);rotation=orientation_error_deg(B,Btarget);tilt=float(np.degrees(np.arccos(np.clip((-B[:,1])@up,-1,1))))
        if np.linalg.norm(C-Ct)*1000>15 or orientation_error_deg(B,Bt)>20:search_stop='waypoint_pose_divergence';break
        if a.staged_slide and phase!='hold':
            pressures={role:0. for role in ('if_tip','th_tip')};height_sums={role:0. for role in pressures}
            for ci,c in enumerate(d.contact):
                x,y=int(c.geom1),int(c.geom2)
                if cube not in (x,y):continue
                other=y if x==cube else x;name=m.geom(other).name
                if name in pressures:
                    ff=np.zeros(6);mujoco.mj_contactForce(m,d,ci,ff);pressures[name]+=float(ff[0]);height_sums[name]+=float(ff[0])*float((c.pos-C)@up)
            pressure_history.append([pressures['if_tip'],pressures['th_tip']]);average_pressure=dict(zip(('if_tip','th_tip'),np.mean(pressure_history[-10:],axis=0)))
            if index%5==0:
                role_gaps={role:min(v for name,v in gaps(m,d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3),gg,True)) for role,gg in role_geometry.items()}
                if phase=='tighten':upper_gap=min(v for name,v in gaps(m,d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3),geometry,True))
            ready=False
            if phase=='unload':ready=index*.075>=2 and all(.1<pressures[role]<.7 for role in pressures) and rotation<3 and position<2
            elif phase=='slide_th':ready=role_gaps.get('th_tip',-100)>.8 and pressures['th_tip']>.1 and rotation<4 and position<2
            elif phase=='slide_if':ready=all(role_gaps.get(role,-100)>.8 and pressures[role]>.1 for role in pressures) and rotation<3 and position<1.5
            elif phase=='tighten':ready=upper_gap is not None and upper_gap>.8 and all(value>.5 for value in pressures.values()) and rotation<3 and tilt<3 and position<1.5
            elif phase=='preload':ready=all(value>1.8 for value in average_pressure.values()) and rotation<4 and position<2
            if a.release_index_base and phase=='slide_th':ready=role_gaps.get('th_tip',-100)>.8 and average_pressure.get('th_tip',0)>.1 and rotation<4 and position<2
            stable=stable+1 if ready else 0
            if stable>=(20 if phase=='tighten' else 10):
                oldphase=phase;phase={'unload':'slide_th','slide_th':('preload' if a.release_index_base else 'slide_if'),'preload':'slide_if','slide_if':'tighten','tighten':'hold'}[phase];stable=0;stage_since=index
                if a.release_index_base and oldphase=='slide_th':held_thumb_height=float(np.clip(height_sums['th_tip']/max(pressures['th_tip'],1e-9),-.014,-.008))
        else:
            pose_ready=phase=='level' and index*.075>=motion_seconds and position<1 and rotation<a.settle_deg and tilt<2
            if a.lower_contacts and pose_ready:upper_gap=min(v for name,v in gaps(m,d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3),geometry,True))
            stable=stable+1 if pose_ready and (not a.lower_contacts or upper_gap>.8) else 0
            if stable>=20:phase='hold'
        if index%20==0 or stop or held>=134:
            trace.append(dict(sample=index,phase=phase,yaw_deg=angle(B,z['initial_cube_rotation'],up),up_tilt_deg=tilt,target_error_deg=rotation,center_error_mm=position,upper_gap_when_pose_ready_mm=upper_gap,role_gaps_mm=role_gaps,contact_pressure_N=pressures,average_pressure_N=average_pressure,held_thumb_height_m=held_thumb_height));print(json.dumps(trace[-1]),flush=True)
        if index%2==1 or stop or held>=134:snapshots.append(d.qpos.copy())
        if stop or held>=134:break
    for k,v in properties.items():assert np.array_equal(getattr(m,k),v)
    np.savez_compressed(out/'suffix.npz',start=start,goals=rows);np.savez_compressed(out/'trajectory.npz',qpos=snapshots);np.savez_compressed(out/'end_state.npz',qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,act=d.act,qacc_warmstart=d.qacc_warmstart,time=d.time,up=up,initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
    constant=bool(len(rows)>=134 and np.array_equal(np.asarray(rows[-134:]),np.tile(rows[-1],(134,1))));verification=run(m,z,rows,1,out/'verified_end_state.npz',out/'verified_trajectory.npz',True,5,('if_tip','th_tip')) if not stop and not search_stop and held>=134 and constant else None;reasons=quarter_reasons(verification,Ctarget,constant,orientation_error_deg(d.xmat[body].reshape(3,3),Btarget));report=dict(status='opposing_index_thumb_level_and_finish90',parameters=vars(a),source_state=source.relative_to(R).as_posix(),source_state_sha256=sha(source),source_model_sha256=sha(model),source_script_sha256=sha(__import__('pathlib').Path(__file__)),samples=len(rows),phase=phase,guard_stop=stop,search_stop=search_stop,final_yaw_deg=angle(d.xmat[body].reshape(3,3),z['initial_cube_rotation'],up),planned_new_common_center_world_m=Ctarget.tolist(),previous_common_center_not_claimed=True,worst=worst,trace=trace,constant134=constant,recorded_goal_replay=verification,checkpoint90_and_new_grip_passed=not reasons,reasons=reasons,perfect_simulated_pose_and_contact_feedback=True,no_external_cube_force_or_pose_written=True,full_initial_placement_not_replayed=True,native_geometry_caps_and_dynamic_guards_unchanged=True,devices_opened=False,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k not in ('trace','recorded_goal_replay')}),flush=True)
if __name__=='__main__':main()
