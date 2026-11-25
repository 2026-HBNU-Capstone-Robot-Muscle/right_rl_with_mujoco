"""Pinch harder, lift off the root support, transfer, turn and set down.

The index root is not commanded to follow the cube during the lift: that
would keep the same support under it and prevent a true transfer. Only task
selection changes. The floor/support collision, native ranges, torque caps,
2tick increments and all1ms dynamic guards are preserved. Offline only.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor,angle
from hand_motor_mapping import ticks_to_sim
from research_quasistatic_wrench_path_20261105 import allocate_native,hybrid_increment
from offline_fractional_goal_accumulator_20261105 import FractionalGoals
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg
from canonical_grip_dynamics_20261105 import run
from checkpoint_quarter_review_20261106 import quarter_reasons

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--station-y-mm',type=float,choices=(4.,8.),required=True);ap.add_argument('--normal-N',type=float,choices=(2.,2.5),default=2.);ap.add_argument('--lift-mm',type=float,choices=(6.,8.),default=6.);ap.add_argument('--samples',type=int,default=900);ap.add_argument('--cap',type=float,choices=(.15,.23),default=.15);a=ap.parse_args()
    if not 134<=a.samples<=900:raise ValueError('bounded offline workload')
    source=ROOT/f'torque_cap_sensitivity/cap{a.cap:g}/end_state.npz';model=source.parent/'model.mjb';z=np.load(source);m=mujoco.MjModel.from_binary_path(str(model));d=restore(m,z);hand,support,check=inspect_for(m);cube=m.geom('cube').id;body=m.body('cube').id;up=z['up'];C0=d.qpos[-7:-4].copy();B0=d.xmat[body].reshape(3,3).copy();center=C0+np.array([0.,a.station_y_mm/1000,0.]);raised0=C0+up*a.lift_mm/1000;raised1=center+up*a.lift_mm/1000;targetB=upright_yaw_target(z['initial_cube_rotation'],up,90);turn_vector=Rotation.from_matrix(targetB@B0.T).as_rotvec();lo,hi,nlo,nhi=native(m);start=motor(d.ctrl);goals=start.copy();planner=FractionalGoals(start,np.minimum(lo+12,start),np.maximum(hi-12,start));references={'mf_tip':a.normal_N,'th_tip':a.normal_N};memory={};ignored=tuple(m.geom(g).name for g in range(m.ngeom) if m.body(int(m.geom_bodyid[g])).name.startswith('if_'));phase='preload';phase_start=0;stable=held=0;rows=[];trace=[];snapshots=[];sample_ids=[];pressure_history=[];stop=search_stop=None;gap=longest=0.;worst=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,range_rad=0.);began=time.perf_counter();tag=f'y{a.station_y_mm:g}_normal{a.normal_N:g}_lift{a.lift_mm:g}'+(f'_cap{a.cap:g}' if a.cap!=.15 else '');out=ROOT/'pinch_transfer_turn'/tag;out.mkdir(parents=True,exist_ok=True);properties={k:getattr(m,k).copy() for k in ('actuator_gainprm','actuator_biasprm','actuator_forcelimited','actuator_forcerange','jnt_range','geom_size','geom_contype','geom_conaffinity')}
    for index in range(1,a.samples+1):
        elapsed=(index-1-phase_start)*.075;C=d.qpos[-7:-4].copy();B=d.xmat[body].reshape(3,3);duration=8. if phase=='yaw' else (4. if phase=='transfer' else 3.);blend=float(np.clip(elapsed/duration,0,1));blend=blend*blend*(3-2*blend);Ct=C0.copy();Bt=B0
        if phase=='lift':Ct=C0+(raised0-C0)*blend
        elif phase=='transfer':Ct=raised0+(raised1-raised0)*blend
        elif phase=='yaw':Ct=raised1;Bt=Rotation.from_rotvec(turn_vector*blend).as_matrix()@B0
        elif phase=='lower':Ct=raised1+(center-raised1)*blend;Bt=targetB
        elif phase=='hold':Ct=center;Bt=targetB
        if phase=='hold':held+=1
        else:
            velocity=np.zeros(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,body,velocity,0);F=-m.body_mass[body]*m.opt.gravity+30*(Ct-C)-1.5*velocity[3:];T=.04*Rotation.from_matrix(Bt@B.T).as_rotvec()-.002*velocity[:3];proposal,allocation=allocate_native(m,d,cube,C,F,T,references,hand,True,(nlo,nhi),('mf_tip','th_tip'),normal_reference_weight=2.)
            if proposal is None:search_stop=allocation['reason'];break
            goals=hybrid_increment(m,d,allocation,planner,C,Ct,B,Bt,tip_memory=memory,world_up=up,maximum_tracking_rad=.12,ignored_motion_geoms=ignored if phase!='preload' else (),recovery_normal_targets=references)
        d.ctrl[:]=ticks_to_sim(goals);rows.append(goals.copy())
        for sub in range(75):
            mujoco.mj_step(m,d);su,sp=check(d);worst['self_mm']=max(worst['self_mm'],max(sp.values(),default=0.));worst['support_mm']=max(worst['support_mm'],max(su.values(),default=0.));worst['tracking_rad']=max(worst['tracking_rad'],float(max(abs(d.qpos[:16]-d.ctrl))));worst['range_rad']=max(worst['range_rad'],float(max(0,max(nlo-d.qpos[:16]),max(d.qpos[:16]-nhi))));inhand=external=False
            for c in d.contact:
                x,y=int(c.geom1),int(c.geom2)
                if cube not in (x,y):continue
                other=y if x==cube else x
                if int(m.geom_bodyid[other]) in hand:inhand|=c.dist<.0004;worst['cube_mm']=max(worst['cube_mm'],max(0.,-float(c.dist)*1000))
                elif other in support and c.dist<.0004:external=True
            gap=0. if inhand else gap+.001;longest=max(longest,gap)
            if not np.isfinite(np.r_[d.qpos,d.qvel]).all() or worst['self_mm']>1e-6 or worst['support_mm']>.15 or worst['cube_mm']>1 or worst['tracking_rad']>.18 or worst['range_rad']>1e-6 or external or longest>.25 or np.linalg.norm(d.qpos[-7:-4]-C0)>.08:stop='original_dynamic_bound';break
        C=d.qpos[-7:-4].copy();B=d.xmat[body].reshape(3,3);position=float(np.linalg.norm(C-Ct)*1000);rotation=orientation_error_deg(B,Bt);pressures={name:0. for name in ('mf_tip','th_tip')};root_load=0.
        for ci,c in enumerate(d.contact):
            x,y=int(c.geom1),int(c.geom2)
            if cube not in (x,y):continue
            other=y if x==cube else x;name=m.geom(other).name;ff=np.zeros(6);mujoco.mj_contactForce(m,d,ci,ff)
            if name in pressures:pressures[name]+=float(ff[0])
            elif name.startswith('if_'):root_load+=float(ff[0])
        pressure_history.append([pressures['mf_tip'],pressures['th_tip']]);average=np.mean(pressure_history[-10:],axis=0)
        ready=elapsed>=duration and position<1 and rotation<4
        if phase=='preload':ready=elapsed>=3 and position<1 and rotation<3 and min(average)>a.normal_N*.85
        if phase=='lift':ready=ready and root_load<.05
        if phase=='lower':ready=ready and float(np.degrees(np.arccos(np.clip((-B[:,1])@up,-1,1))))<3
        stable=stable+1 if ready and phase!='hold' else 0
        if stable>=15:phase={'preload':'lift','lift':'transfer','transfer':'yaw','yaw':'lower','lower':'hold'}[phase];phase_start=index;stable=0
        if np.linalg.norm(C-Ct)*1000>15 or orientation_error_deg(B,Bt)>20:search_stop='waypoint_pose_divergence'
        if index%20==0 or stop or search_stop or held>=134:
            trace.append(dict(sample=index,phase=phase,yaw_deg=angle(B,z['initial_cube_rotation'],up),up_tilt_deg=float(np.degrees(np.arccos(np.clip((-B[:,1])@up,-1,1)))),position_error_mm=position,orientation_error_deg=rotation,contact_normal_N=pressures,average_contact_normal_N=average.tolist(),index_support_load_N=root_load));print(json.dumps(trace[-1]),flush=True)
        if index%2 or stop or search_stop or held>=134:snapshots.append(d.qpos.copy());sample_ids.append(index)
        if stop or search_stop or held>=134:break
    for k,value in properties.items():assert np.array_equal(getattr(m,k),value),k
    np.savez_compressed(out/'suffix.npz',start=start,goals=rows);np.savez_compressed(out/'trajectory.npz',qpos=snapshots,goal_sample=sample_ids);np.savez_compressed(out/'end_state.npz',qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,act=d.act,qacc_warmstart=d.qacc_warmstart,time=d.time,up=up,initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center']);constant=bool(len(rows)>=134 and np.array_equal(np.asarray(rows[-134:]),np.tile(rows[-1],(134,1))));verification=run(m,z,rows,1,out/'verified_end_state.npz',out/'verified_trajectory.npz',True) if not stop and not search_stop and held>=134 and constant else None;why=quarter_reasons(verification,center,constant,orientation_error_deg(d.xmat[body].reshape(3,3),targetB));report=dict(status='pinch_lift_transfer_turn_setdown',parameters=vars(a),source_state=source.relative_to(R).as_posix(),source_state_sha256=sha(source),source_model_sha256=sha(model),phase=phase,samples=len(rows),guard_stop=stop,search_stop=search_stop,worst=worst,trace=trace,planned_new_common_center_world_m=center.tolist(),recorded_goal_replay=verification,constant134=constant,checkpoint90_and_new_grip_passed=not why,reasons=why,full_initial_placement_not_replayed=True,cube_never_reset_or_forced=True,native_geometry_and_dynamic_guards_unchanged=True,devices_opened=False,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k not in ('trace','recorded_goal_replay')}),flush=True)
if __name__=='__main__':main()
