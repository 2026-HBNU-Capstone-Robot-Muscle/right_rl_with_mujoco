"""Clear IF/RF upper sweep while retaining the rechecked RF/base-supported hold.

IK fixes the cube only in proposal copies. Every transition/hold uses one free
model state; strict final-window geometry is checked from its recorded poses.
"""
from pathlib import Path
import time
import json
from collections import deque
import numpy as np
import mujoco
from scipy.optimize import least_squares
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,TICK,SIM_TO_MOTOR_IDS,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    folder=OUT/'next_cycle/handoff_refinement_20261110/RF_work_grip_refinement'
    if folder.exists():raise RuntimeError('Preserve previous refinement')
    model,initial,_,_,provenance=assets('active_ccd50');cube=model.geom('cube').id
    source=OUT/'next_cycle/handoff_refinement_20261110/RF_endpoint_CCD50_load_screen/qualified0_end_state.npz'
    z=dict(np.load(source,allow_pickle=False));d=restore(model,z);guard=Guard(model)
    C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();lag=d.ctrl-d.qpos[:16]
    geometry=precompute(model,guard.hand);rf=np.arange(8,12);free=np.r_[np.arange(1,4),rf]
    moving=[item for item in geometry if (model.body(int(model.geom_bodyid[item[0]])).name or '').startswith(('if_','rf_'))]
    sid=np.array(SIM_TO_MOTOR_IDS)-1
    low=np.maximum(guard.qlo,(np.ceil(guard.lo[sid])+2-2048)*TICK-lag)
    high=np.minimum(guard.qhi,(np.floor(guard.hi[sid])-2-2048)*TICK-lag)
    tip=model.geom('rf_tip').id
    touching=[contact for contact in d.contact if {int(contact.geom1),int(contact.geom2)}=={cube,tip}]
    if not touching:raise RuntimeError('The selected RF source no longer contacts the cube')
    point=np.mean([contact.pos for contact in touching],axis=0);local_tip=d.geom_xmat[tip].reshape(3,3).T@(point-d.geom_xpos[tip])
    cube_point=B.T@(point-C)*1000;axis=max((0,2),key=lambda index:abs(cube_point[index]));across=2 if axis==0 else 0
    targets=[]
    for height in (8.,14.,21.):
        for offset in (-14.,0.,14.):
            target=cube_point.copy();target[axis]=np.sign(target[axis])*27.4;target[1]=height;target[across]=offset
            targets.append(target)
    seeds=[d.qpos[free].copy(),d.qpos[free].copy()]
    seeds[1][:3]=np.clip(0.,low[free[:3]],high[free[:3]])
    folder.mkdir();(folder/'source_refine_RF_work_grip.py').write_bytes(Path(__file__).read_bytes())
    candidates=[];fits=[];began=time.perf_counter()
    for target_index,local_target in enumerate(targets):
        target=C+B@local_target/1000
        for seed_index,seed in enumerate(seeds):
            p=restore(model,z)
            def residual(x):
                p.qpos[:]=z['qpos'];p.qpos[free]=x;p.ctrl[:]=p.qpos[:16]+lag;p.qvel[:]=0
                mujoco.mj_forward(model,p)
                actual=p.geom_xpos[tip]+p.geom_xmat[tip].reshape(3,3)@local_tip
                support,own=guard.contacts(p)
                penetration=max((-float(c.dist)*1000 for c in p.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
                clear=np.array([max(0.,.6-value) for _,value in gaps(model,p,C,B,moving,True)])
                return np.r_[(actual-target)*1000,(x-seed)*.04,clear*5,
                    max(own.values(),default=0.)*1000,max(support.values(),default=0.)*1000,max(0.,penetration-.8)*300]
            fit=least_squares(residual,np.clip(seed,low[free],high[free]),bounds=(low[free],high[free]),max_nfev=100,
                              ftol=1e-7,xtol=1e-7,gtol=1e-7)
            error=float(np.linalg.norm(residual(fit.x)[:3]));support,own=guard.contacts(p)
            contact=any(cube in (int(c.geom1),int(c.geom2)) and
                (model.body(int(model.geom_bodyid[int(c.geom2) if int(c.geom1)==cube else int(c.geom1)])).name or '').startswith('rf_') and c.dist<.0002
                for c in p.contact)
            goal=motor(p.ctrl);screen=min(value for _,value in gaps(model,p,C,B,geometry,True))
            possible=bool(contact and error<2 and not own and max(support.values(),default=0.)<=.15 and
                          np.all(goal>=guard.lo) and np.all(goal<=guard.hi) and screen>.3)
            strict=None
            if possible:
                certificate=SweepCertificate(model,guard.hand,.3);value,bad=certificate.query(p,C,B)
                strict=dict(lower_mm=value,bad_geoms=[model.geom(int(certificate.ids[i])).name for i in bad])
                possible=not bad and value>.3
            row=dict(target_index=target_index,target_cube_frame_mm=local_target.tolist(),seed=seed_index,
                     contact_point_error_mm=error,self_pairs_mm=own,upper_screen_mm=screen,
                     strict_certificate=strict,static_work_grip_candidate=bool(possible),goal_ticks=goal.tolist())
            fits.append(row)
            if possible:
                candidates.append((goal.copy(),p.qpos.copy(),p.ctrl.copy(),len(fits)-1))
                np.savez_compressed(folder/f'proposal{len(fits)-1}.npz',qpos=p.qpos,ctrl=p.ctrl)
            print(json.dumps(dict(fit=len(fits),static_candidate=bool(possible),screen_mm=screen,point_error_mm=error)),flush=True)
    selected=[]
    for candidate in candidates:
        if all(np.max(abs(candidate[0]-old[0]))>20 for old in selected):selected.append(candidate)
        if len(selected)==2:break
    target_rotation=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90)
    station=np.load(OUT/'baseline_archived_zero/end_state.npz',allow_pickle=False)['qpos'][-7:-4]
    trials=[]
    for base_goal,pose,control,fit_index in selected:
        prep=restore(model,z);prep.qpos[:]=pose;prep.ctrl[:]=control;mujoco.mj_forward(model,prep)
        rf_contact=next(c for c in prep.contact if {int(c.geom1),int(c.geom2)}=={cube,tip})
        normal=rf_contact.frame[:3]*(1 if int(rf_contact.geom1)==cube else -1)
        J=np.zeros((3,model.nv));mujoco.mj_jac(model,prep,J,None,rf_contact.pos,int(model.geom_bodyid[tip]))
        closing=-J[:,:16].T@normal/model.actuator_gainprm[:,0]
        for preload in (0.,.4):
            desired=control.copy();desired[rf]+=closing[rf]*preload;goal=motor(desired)
            if np.any(goal<guard.lo) or np.any(goal>guard.hi):continue
            for order in ('together','index_then_ring'):
                data=restore(model,z);checker=Guard(model);previous=motor(data.ctrl);rows=[];stop=[]
                groups=[free] if order=='together' else [np.arange(1,4),rf]
                for group in groups:
                    before=previous.copy();after=before.copy();after[sid[group]]=goal[sid[group]]
                    count=max(1,int(np.ceil(np.max(abs(after-before))/2)))
                    for step in range(count):
                        commanded=np.rint(before+(after-before)*(step+1)/count).astype(int)
                        if np.max(abs(commanded-previous))>2+1e-9:raise AssertionError('Rate changed')
                        data.ctrl[:]=ticks_to_sim(commanded);previous=commanded;rows.append(commanded.copy())
                        for _ in range(75):
                            mujoco.mj_step(model,data);stop=checker.check(data)
                            if np.linalg.norm(data.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                            if stop:break
                        if stop:break
                    if stop:break
                history=deque(maxlen=4050);supported=rf_supported=samples=0;hold_B=hold_C=None;rotation_drift=position_drift=0.
                if not stop:
                    for control_step in range(134):
                        rows.append(previous.copy())
                        for _ in range(75):
                            mujoco.mj_step(model,data);stop=checker.check(data)
                            if np.linalg.norm(data.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                            if control_step>=80:
                                sample=sample_model(model,data,checker.hand,cube);samples+=1;supported+=sample['supported_sample']
                                rf_supported+=sum(v['normal_N'] for name,v in sample['by_geom'].items()
                                    if (model.body(int(model.geom_bodyid[model.geom(name).id])).name or '').startswith('rf_'))>.2
                                current_B=data.geom_xmat[cube].reshape(3,3);current_C=data.geom_xpos[cube]
                                if hold_B is None:hold_B=current_B.copy();hold_C=current_C.copy()
                                rotation_drift=max(rotation_drift,orientation_error_deg(current_B,hold_B))
                                position_drift=max(position_drift,float(np.linalg.norm(current_C-hold_C)*1000))
                                history.append(data.qpos.copy())
                            if stop:break
                        if stop:break
                final_error=orientation_error_deg(data.geom_xmat[cube].reshape(3,3),target_rotation)
                tilt=face_up_error_deg(data.geom_xmat[cube].reshape(3,3),5,initial['up'])
                station_error=float(np.linalg.norm(data.geom_xpos[cube]-station)*1000)
                base_pass=bool(not stop and samples==4050 and supported/4050>.8 and rf_supported/4050>.8 and
                               rotation_drift<3 and position_drift<2 and final_error<5 and tilt<5 and station_error<2)
                cert=SweepCertificate(model,checker.hand,.3);certified=0;gap=None;bad=[]
                if base_pass:
                    geometry_data=restore(model,dict(z,qpos=history[-1],qvel=np.zeros(model.nv)))
                    gap,bad=cert.query(geometry_data,geometry_data.geom_xpos[cube],geometry_data.geom_xmat[cube].reshape(3,3))
                    if not bad and gap>.3:
                        for qpos in history:
                            geometry_data.qpos[:]=qpos;mujoco.mj_forward(model,geometry_data)
                            value,bad=cert.query(geometry_data,geometry_data.geom_xpos[cube],geometry_data.geom_xmat[cube].reshape(3,3))
                            gap=min(gap,value);certified+=1
                            if bad or gap<=.3:break
                passed=bool(base_pass and certified==4050 and not bad and gap>.3)
                label=f'fit{fit_index}_preload{preload:g}_{order}';trial=folder/label;trial.mkdir()
                np.savez_compressed(trial/'research_goals.npz',start=motor(z['ctrl']),goals=rows)
                np.savez_compressed(trial/'end_state.npz',**{key:getattr(data,key) for key in BOUND},time=data.time,
                    up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
                result=dict(fit=fit_index,preload_force_hypothesis_N=preload,order=order,steps=len(rows),stop=stop,
                    final_global90_error_deg=final_error,final_tilt_deg=tilt,common_station_error_mm=station_error,
                    hold_samples=samples,whole_hand_support_fraction=supported/max(1,samples),RF_support_fraction=rf_supported/max(1,samples),
                    hold_angle_drift_deg=rotation_drift,hold_center_drift_mm=position_drift,pose_and_load_passed=base_pass,
                    certified_window_samples=certified,upper_certificate_lower_bound_mm=gap,initialized_RF_clearance_suffix_passed=passed,
                    geometry_recheck_uses_recorded_free_trajectory_not_substituted_motion=True,worst=checker.worst,
                    actual_model_state_not_reset_during_suffix=True,first_rotation_from_initial_placement_not_validated=True)
                save_json(trial/'review.json',result);trials.append(dict(path=trial.relative_to(OUT).as_posix(),**result))
                print(json.dumps(dict(trial=label,steps=len(rows),stop=stop,passed=passed,error_deg=final_error)),flush=True)
    report=dict(status='RF_supported_work_grip_clearance_refinement',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),
        model_provenance=provenance,index_base_RF_thumb_and_palm_support_are_not_replaced_by_two_tip_requirement=True,
        free_sim_indices=free.tolist(),index_base_joint_held=True,thumb_and_middle_commands_held=True,
        fits=fits,static_candidates=len(candidates),dynamic_trials=trials,
        initialized_clear_work_grip_suffix_passed=any(row['initialized_RF_clearance_suffix_passed'] for row in trials),
        full_initial_placement_two_turn_cycle_passed=False,new_SAC_training_steps=0,
        devices_opened=0,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began)
    save_json(folder/'review.json',report)
    print(json.dumps(dict(fits=len(fits),static_candidates=len(candidates),dynamic_trials=len(trials),passed=report['initialized_clear_work_grip_suffix_passed'])),flush=True)


if __name__=='__main__':main()
