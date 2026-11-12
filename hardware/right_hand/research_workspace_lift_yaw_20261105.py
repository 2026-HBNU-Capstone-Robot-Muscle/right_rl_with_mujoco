"""Offline free-cube lift and yaw with bounded simulated contact feedback.

This is a research controller with perfect simulated object/contact feedback,
not a robot policy. Cube pose is restored once and is never prescribed again.
Motor adapters, camera adapters and live execution are deliberately absent.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse, hashlib, json, time
import numpy as np
import mujoco
from scipy.optimize import lsq_linear
from dynamic_common_grip_search_20261105 import ROOT, R, CP
from research_rotation_body_hold_20261104 import restore, inspect_for, native, motor, angle
from hand_motor_mapping import TICK, SIM_TO_MOTOR_IDS, ticks_to_sim
from convex_distance_check_20261104 import CheckedDistance
from offline_fractional_goal_accumulator_20261105 import FractionalGoals

A=np.r_[np.arange(4,8),np.arange(12,16)]
IDS=np.asarray(SIM_TO_MOTOR_IDS)-1

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--force',type=float,choices=(1.5,2.5,3.5),default=2.5)
    ap.add_argument('--lift-mm',type=float,choices=(8.,16.,24.),default=16.)
    ap.add_argument('--rotation-weight',type=float,choices=(0.,5.,20.),default=5.)
    ap.add_argument('--samples',type=int,default=900)
    ap.add_argument('--support-index',action='store_true')
    ap.add_argument('--origin',choices=('workspace','common'),default='workspace')
    a=ap.parse_args()
    if not 300<=a.samples<=1200:raise ValueError('bounded offline workload')
    folder=ROOT/f'workspace_transition/xpos_forward/force{a.force:g}' if a.origin=='workspace' else ROOT/'full_placement_v3_thumb12/front-10/g1_f0.3'
    source=folder/'end_state.npz'; z=np.load(source)
    prefix=np.load(folder/'candidate.npz' if a.origin=='workspace' else ROOT/'tip_squeeze/id1/plus12/candidate.npz')
    m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb' if a.origin=='workspace' else folder/'model.mjb'))
    d=restore(m,z); cube=m.geom('cube').id; body=m.body('cube').id
    start=motor(d.ctrl); assert np.array_equal(start,prefix['goals'][-1])
    active=np.r_[np.arange(8),np.arange(12,16)] if a.support_index else A
    out=ROOT/f'workspace_lift_yaw/force{a.force:g}_lift{a.lift_mm:g}_rotation{a.rotation_weight:g}{"_index_support" if a.support_index else ""}{"_from_common" if a.origin=="common" else ""}'
    out.mkdir(parents=True,exist_ok=True)
    hand,support,check=inspect_for(m);lo,hi,nlo,nhi=native(m)
    planner=FractionalGoals(start,np.ceil(lo)+4,np.floor(hi)-4)
    tips=[m.geom(name).id for name in ('mf_tip','th_tip')]
    if a.support_index:
        supported={}
        for ci,c in enumerate(d.contact):
            x,y=int(c.geom1),int(c.geom2)
            if cube not in (x,y):continue
            g=y if x==cube else x
            if (m.geom(g).name or '').startswith('if_'):
                force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force);supported[g]=supported.get(g,0.)+float(force[0])
        if not supported:raise ValueError('source has no supporting index contact')
        tips.append(max(supported,key=supported.get))
    checked=CheckedDistance(m,cube,tips)
    C0=d.qpos[-7:-4].copy();up=z['up'];Binit=z['initial_cube_rotation']
    target_center=C0+up*a.lift_mm/1000
    target_yaw=90.; mode='lift'; stable=0; hold=0;stop=None;search_stop=None
    rows=[]; phases=[]; trace=[];snapshots=[];snapshot_samples=[]
    gap=longest=0.; force_reads=[]; began=time.perf_counter()
    yaw=angle(d.xmat[body].reshape(3,3),Binit,up)
    worst=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,range_rad=0.)
    properties={name:getattr(m,name).copy() for name in ('actuator_forcerange','actuator_gainprm','actuator_biasprm','geom_size','geom_contype','geom_conaffinity')}
    for index in range(1,a.samples+1):
        C=d.qpos[-7:-4].copy();B=d.xmat[body].reshape(3,3)
        measured={g:0. for g in tips}
        for ci,c in enumerate(d.contact):
            x,y=int(c.geom1),int(c.geom2)
            if cube not in (x,y):continue
            other=y if x==cube else x
            if other in measured:
                force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force);measured[other]+=float(force[0])
        if mode=='hold':
            hold+=1;goal=planner.goal.copy()
        else:
            velocity=(target_center-C)*.06
            length=np.linalg.norm(velocity)
            if length>.00015:velocity*=.00015/length
            omega=np.cross(-B[:,1],up)*.03
            if mode=='yaw':omega+=up*float(np.clip(np.radians(target_yaw-yaw)*.1,-.0015,.0015))
            normal_j=[];normal_goal=[];tangent_j=[];tangent_goal=[];rot_j=[];rot_goal=[]
            for role,g in enumerate(tips):
                contacts=[c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,g}]
                if contacts:
                    c=min(contacts,key=lambda c:c.dist);normal=c.frame[:3]*(1 if int(c.geom1)==cube else -1);point=c.pos.copy()
                else:
                    try:distance,pair=checked.distance(d,g)
                    except RuntimeError as exc:
                        search_stop='contact_distance_not_certified: '+str(exc);break
                    if abs(distance)<=1e-9:search_stop='undefined_simulated_contact_normal';break
                    normal=(pair[3:]-pair[:3])/distance;normal/=np.linalg.norm(normal);point=pair[3:]
                jp=np.zeros((3,m.nv));jr=np.zeros((3,m.nv))
                mujoco.mj_jac(m,d,jp,jr,point,int(m.geom_bodyid[g]))
                jac=jp[:,active];project=np.eye(3)-np.outer(normal,normal)
                # Incremental position targets preserve the existing actuator
                # stiffness. Force error only proposes bounded goal changes.
                compliance=float(np.sum((normal@jac)**2/m.actuator_gainprm[active,0]))
                closure=0. if role==2 else float(np.clip((a.force-measured[g])*compliance*.05,-.00008,.00008))
                desired=velocity+np.cross(omega,point-C)
                normal_j.append(normal@jac);normal_goal.append(float(normal@desired)-closure)
                tangent_j.append(project@jac);tangent_goal.append(project@desired)
                if role<2:rot_j.append(np.column_stack([np.cross(jr[:,j],normal) for j in active]));rot_goal.append(np.cross(omega,normal))
            if search_stop:break
            matrix=np.vstack([np.array(normal_j)*10000,np.vstack(tangent_j)*2000,np.eye(len(active))*.02])
            rhs=np.r_[np.array(normal_goal)*10000,np.concatenate(tangent_goal)*2000,np.zeros(len(active))]
            if a.rotation_weight:
                matrix=np.vstack([matrix,np.vstack(rot_j)*a.rotation_weight]);rhs=np.r_[rhs,np.concatenate(rot_goal)*a.rotation_weight]
            lower,upper=planner.increment_bounds();lower=lower[IDS[active]]*TICK;upper=upper[IDS[active]]*TICK
            fixed=upper-lower<1e-12;change=np.zeros(len(active));change[fixed]=(lower[fixed]+upper[fixed])/2
            free=~fixed
            if np.any(free):change[free]=lsq_linear(matrix[:,free],rhs-matrix[:,fixed]@change[fixed],bounds=(lower[free],upper[free]),tol=1e-10,max_iter=100).x
            delta=np.zeros(16);delta[IDS[active]]=change/TICK;goal=planner.advance(delta)
        d.ctrl[:]=ticks_to_sim(goal);rows.append(goal.copy());phases.append(mode)
        for sub in range(75):
            mujoco.mj_step(m,d);su,sp=check(d)
            worst['self_mm']=max(worst['self_mm'],max(sp.values(),default=0.))
            worst['support_mm']=max(worst['support_mm'],max(su.values(),default=0.))
            worst['tracking_rad']=max(worst['tracking_rad'],float(max(abs(d.qpos[:16]-d.ctrl))))
            worst['range_rad']=max(worst['range_rad'],float(max(0,max(nlo-d.qpos[:16]),max(d.qpos[:16]-nhi))))
            inhand=external=False
            for c in d.contact:
                x,y=int(c.geom1),int(c.geom2)
                if cube not in (x,y):continue
                other=y if x==cube else x
                if int(m.geom_bodyid[other]) in hand:
                    inhand|=c.dist<.0004;worst['cube_mm']=max(worst['cube_mm'],max(0.,-float(c.dist)*1000))
                elif other in support and c.dist<.0004:external=True
            gap=0. if inhand else gap+.001;longest=max(longest,gap)
            if (not np.isfinite(np.r_[d.qpos,d.qvel,d.ctrl]).all() or worst['self_mm']>1e-6 or worst['support_mm']>.15 or worst['cube_mm']>1 or worst['tracking_rad']>.18 or worst['range_rad']>1e-6 or external or longest>.25 or np.linalg.norm(d.qpos[-7:-4]-C0)>.08):
                stop='unchanged contact/range/tracking/retention bound';break
        B=d.xmat[body].reshape(3,3);C=d.qpos[-7:-4].copy();yaw=angle(B,Binit,up)
        tilt=float(np.degrees(np.arccos(np.clip((-B[:,1])@up,-1,1))))
        error=float(np.linalg.norm(C-target_center)*1000)
        if mode=='lift':
            stable=stable+1 if error<.75 and tilt<5 else 0
            if stable>=20:mode='yaw';stable=0
        elif mode=='yaw':
            stable=stable+1 if error<1 and abs(yaw-target_yaw)<1 and tilt<5 else 0
            if stable>=20:mode='hold';stable=0
        if index%2==1 or stop or hold>=134:snapshots.append(d.qpos.copy());snapshot_samples.append(index)
        if index%40==0 or stop or hold>=134:
            row=dict(sample=index,phase=mode,yaw_deg=yaw,tilt_deg=tilt,lift_mm=float((C-C0)@up*1000),center_error_mm=error,contact_force_N={m.geom(g).name:v for g,v in measured.items()},goal=goal.tolist());trace.append(row)
            print(json.dumps(dict(sample=index,phase=mode,yaw=yaw,tilt=tilt,lift=row['lift_mm'],error=error,stop=stop)),flush=True)
        if stop or hold>=134:break
    for name,value in properties.items():assert np.array_equal(getattr(m,name),value)
    np.savez_compressed(out/'suffix.npz',start=start,goals=rows,phases=phases)
    np.savez_compressed(out/'trajectory.npz',samples=snapshot_samples,qpos=snapshots,goal_period_s=.075,top_face_id=5)
    np.savez_compressed(out/'end_state.npz',qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,act=d.act,qacc_warmstart=d.qacc_warmstart,time=d.time,up=up,initial_cube_rotation=Binit,initial_cube_center=z['initial_cube_center'])
    report=dict(status='offline_free_cube_contact_feedback_lift_yaw',parameters=vars(a),source_state=str(source.relative_to(R)),source_state_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),samples=len(rows),safety_passed=stop is None,guard_stop=stop,search_stop=search_stop,final_phase=mode,final_yaw_deg=yaw,final_tilt_deg=tilt,final_lift_mm=float((C-C0)@up*1000),center_error_to_lift_target_mm=error,hold_samples=hold,goal90_and_hold_reached=stop is None and hold>=134 and abs(yaw-90)<5 and tilt<5,worst=worst,trace=trace,elapsed_seconds=time.perf_counter()-began,perfect_simulated_object_and_contact_force_feedback=True,cube_pose_never_reset_after_initial_restore=True,full_initial_placement_not_replayed=True,actuator_collision_properties_and_original_dynamic_bounds_unchanged=True,hardware_execution_allowed=False,devices_opened=False,live_SAC=False)
    report['safety_passed']=stop is None and search_stop is None
    report['unresolved_geometry_query_is_not_a_pass']=True
    report['index_lifting_contact']=m.geom(tips[-1]).name if a.support_index else None
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='trace'}),flush=True)
if __name__=='__main__':main()
