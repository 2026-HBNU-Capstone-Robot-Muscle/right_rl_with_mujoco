"""Privileged MF pressure teacher after a positive-yaw contact arrival.

Support may transfer between all hand parts. Rotation/hold and fixed-station
metrics are reported separately; no original station failure is reclassified.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,SIM_TO_MOTOR_IDS,save_json,sha,angle
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate
from servo_MF_temporary_support import mf_tick_increment


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'actual_A_next_MF_yaw_pressure';source=base/'actual_A_next_MF_positive_yaw_arrival/axis2_sign-1_force0.6/end_state.npz'
    if folder.exists():raise RuntimeError('Preserve directed yaw pressure trials')
    z=dict(np.load(source,allow_pickle=False));model,initial,_,_,provenance=assets('active_ccd50');cube=model.geom('cube').id;tip=model.geom('mf_tip').id;active=np.arange(4,8);sid=np.asarray(SIM_TO_MOTOR_IDS)-1;target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],180.)
    station=np.load(base/'reached_first90_reverse_quarter_probe/first90_space_hold_state.npz',allow_pickle=False)['qpos'][-7:-4]
    folder.mkdir();(folder/'source_servo_directed_yaw_contact.py').write_bytes(Path(__file__).read_bytes());(folder/'source_servo_MF_temporary_support.py').write_bytes((OUT/'servo_MF_temporary_support.py').read_bytes());rows=[]
    for pressure in (.5,1.,1.5):
        sub=folder/f'force{pressure:g}';sub.mkdir();d=restore(model,z);g=Guard(model);start=motor(d.ctrl);previous=start.copy();fraction=np.zeros(4);commands=[];trace=[];trajectory=[d.qpos.copy()];stop=[];B0=d.geom_xmat[cube].reshape(3,3).copy();previousB=B0.copy();phase='turn';hold_count=supported=certified=0;minimum=float('inf');holdC=holdB=None;shift=drift=0.;best=float('inf')
        for step in range(500):
            B=d.geom_xmat[cube].reshape(3,3);error=orientation_error_deg(B,target);best=min(best,error);remaining=float(np.degrees(Rotation.from_matrix(target@B.T).as_rotvec()@initial['up']));yaw_rate=float(np.degrees(Rotation.from_matrix(B@previousB.T).as_rotvec()@initial['up'])/.075);previousB=B.copy()
            if phase=='turn' and error<3. and face_up_error_deg(B,5,initial['up'])<5:phase='release_and_hold';holdC=d.geom_xpos[cube].copy();holdB=B.copy()
            load=sample_model(model,d,g.hand,cube);force=sum(v['normal_N'] for n,v in load['by_geom'].items() if (model.body(int(model.geom_bodyid[model.geom(n).id])).name or '').startswith('mf_'))
            desired=0. if phase=='release_and_hold' or remaining<0 else (pressure if remaining>8 else min(pressure,.45))
            delta,_=mf_tick_increment(model,d,cube,tip,desired,force);fraction+=delta;change=np.clip(np.rint(fraction),-2,2);command=previous.copy();command[sid[active]]=np.clip(previous[sid[active]]+change,np.ceil(g.lo[sid[active]]),np.floor(g.hi[sid[active]]));fraction-=command[sid[active]]-previous[sid[active]]
            assert np.max(abs(command-previous))<=2+1e-9 and np.all(command>=g.lo) and np.all(command<=g.hi);d.ctrl[active]=ticks_to_sim(command)[active];assert np.array_equal(d.ctrl[:4],z['ctrl'][:4]) and np.array_equal(d.ctrl[8:],z['ctrl'][8:]);commands.append(command.copy());previous=command.copy()
            for _ in range(75):
                mujoco.mj_step(model,d);stop=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if phase=='release_and_hold':
                    hold_count+=1;supported+=sample_model(model,d,g.hand,cube)['supported_sample'];C=d.geom_xpos[cube];curB=d.geom_xmat[cube].reshape(3,3);shift=max(shift,float(np.linalg.norm(C-holdC)*1000));drift=max(drift,orientation_error_deg(curB,holdB))
                if stop:break
            trajectory.append(d.qpos.copy());trace.append(dict(step=step+1,phase=phase,error_deg=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),target),pressure_objective_N=desired,MF_force_N_before_step=force,yaw_rate_deg_per_s_before_step=yaw_rate,whole_hand_supported=sample_model(model,d,g.hand,cube)['supported_sample']))
            if stop or hold_count>=4050:break
        # This is an endpoint-only upper-space query. The start state and saved
        # 75ms commands permit independent replay of all1ms hold poses later.
        certificate= SweepCertificate(model,g.hand,.3);lower,bad=certificate.query(d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3))
        error=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),target);center=float(np.linalg.norm(d.geom_xpos[cube]-station)*1000);tilt=face_up_error_deg(d.geom_xmat[cube].reshape(3,3),5,initial['up'])
        rotation_hold=bool(not stop and hold_count==4050 and supported/4050>.8 and error<5 and tilt<5 and shift<2 and drift<3)
        np.savez_compressed(sub/'research_goals.npz',start=start,goals=commands);np.savez_compressed(sub/'trajectory.npz',qpos=trajectory,control_seconds=.075);np.savez_compressed(sub/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        row=dict(path=sub.relative_to(OUT).as_posix(),pressure_hypothesis_N=pressure,steps=len(commands),stop=stop,phase=phase,best180_error_deg=best,final180_error_deg=error,yaw_advance_deg=angle(d.geom_xmat[cube].reshape(3,3),B0,initial['up']),reference_A_station_error_mm=center,reference_A_station_2mm_passed=center<2,hold_samples=hold_count,whole_hand_hold_support_fraction=supported/max(1,hold_count),hold_shift_mm=shift,hold_drift_deg=drift,rotation_and_whole_hand_hold_passed=rotation_hold,final_pose_upper_lower_mm=lower,final_pose_upper_bad_geoms=[model.geom(int(certificate.ids[i])).name for i in bad],full4050_upper_certificate_not_yet_replayed=True,trace=trace,worst=g.worst,terminal_guard=g.last)
        rows.append(row);save_json(sub/'review.json',row);print(json.dumps({k:v for k,v in row.items() if k!='trace'}),flush=True)
    save_json(folder/'review.json',dict(status='directed_MF_pressure_model_teacher',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),feedback_helper_sha256=sha(folder/'source_servo_MF_temporary_support.py'),model_provenance=provenance,rows=rows,only_MF_commands_changed=True,other_hand_parts_may_support=True,contact_identity_not_a_global_invariant=True,privileged_model_distances_and_forces=True,reference_A_2mm_result_reported_separately=True,original_guards_and_bounds_preserved=True,full_cycle_not_certified=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
