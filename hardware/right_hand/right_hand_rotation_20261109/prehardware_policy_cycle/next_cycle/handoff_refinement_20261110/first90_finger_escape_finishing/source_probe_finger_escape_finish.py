"""Move out of the MF/RF crossing corridor before returning to the work grip."""
from pathlib import Path
from collections import deque
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,SIM_TO_MOTOR_IDS,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'first90_finger_escape_finishing'
    if folder.exists():raise RuntimeError('Preserve previous escape study')
    source=base/'first90_finish_checkpoint/state.npz';reference_path=base/'RF_clear_work_grip_joined_suffix/end_state.npz'
    model,initial,_,_,provenance=assets('active_ccd50');cube=model.geom('cube').id;guard=Guard(model)
    z=dict(np.load(source,allow_pickle=False));reference=dict(np.load(reference_path,allow_pickle=False));start=motor(z['ctrl']);target=motor(reference['ctrl'])
    sid=np.array(SIM_TO_MOTOR_IDS)-1;middle=sid[4:8];ring=sid[8:12];all_ids=np.arange(16)
    pool=np.load(base/'early_CCD50_states/state_pool.npz',allow_pickle=False);park=motor(pool['ctrl'][int(np.flatnonzero(pool['indices']==1700)[0])])
    def at(previous,group,values):
        goal=previous.copy();goal[group]=values[group];return goal
    candidates=[]
    for degrees in (20.,45.):
        escape=start.copy();escape[sid[9]]=int(round(2048+degrees*4096/360))
        mf=at(escape,middle,target);rf=at(mf,ring,target)
        candidates.append((f'RF_abduct{degrees:g}_then_MF_then_RF',[escape,mf,rf,target]))
    escape=at(start,ring,park);mf=at(escape,middle,target);rf=at(mf,ring,target)
    candidates.append(('RF_park_then_MF_then_RF',[escape,mf,rf,target]))
    escape=start.copy();escape[sid[5]]=int(round(2048+55*4096/360))
    extend=at(escape,sid[[4,6,7]],target);rf=at(extend,ring,target)
    candidates.append(('MF_abduct55_then_extend_then_RF',[escape,extend,rf,target]))
    folder.mkdir();(folder/'source_probe_finger_escape_finish.py').write_bytes(Path(__file__).read_bytes());rows=[]
    target_rotation=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90)
    canonical=np.load(OUT/'baseline_archived_zero/end_state.npz',allow_pickle=False)['qpos'][-7:-4]
    for name,waypoints in candidates:
        data=restore(model,z);check=Guard(model);previous=start.copy();commands=[];stop=[];phase_results=[]
        for phase,waypoint in enumerate(waypoints):
            if np.any(waypoint<check.lo) or np.any(waypoint>check.hi):stop=['waypoint_native_range'];break
            before=previous.copy();count=max(1,int(np.ceil(np.max(abs(waypoint-before))/2)))
            for step in range(count):
                command=np.rint(before+(waypoint-before)*(step+1)/count).astype(int)
                if np.max(abs(command-previous))>2:raise AssertionError('Rate changed')
                previous=command;data.ctrl[:]=ticks_to_sim(command);commands.append(command.copy())
                for _ in range(75):
                    mujoco.mj_step(model,data);stop=check.check(data)
                    if np.linalg.norm(data.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                    if stop:break
                if stop:break
            phase_results.append(dict(phase=phase,completed=not stop,after_goals=len(commands),stop=stop,guard=check.last))
            if stop:break
        samples=supported=0;history=deque(maxlen=4050);hold_B=hold_C=None;drift=shift=0.
        if not stop:
            for cycle in range(134):
                commands.append(previous.copy())
                for _ in range(75):
                    mujoco.mj_step(model,data);stop=check.check(data)
                    if np.linalg.norm(data.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                    if cycle>=80:
                        samples+=1;supported+=sample_model(model,data,check.hand,cube)['supported_sample'];C=data.geom_xpos[cube];B=data.geom_xmat[cube].reshape(3,3)
                        if hold_B is None:hold_B=B.copy();hold_C=C.copy()
                        drift=max(drift,orientation_error_deg(B,hold_B));shift=max(shift,float(np.linalg.norm(C-hold_C)*1000));history.append(data.qpos.copy())
                    if stop:break
                if stop:break
        C=data.geom_xpos[cube];B=data.geom_xmat[cube].reshape(3,3);error=orientation_error_deg(B,target_rotation);tilt=face_up_error_deg(B,5,initial['up']);center_error=float(np.linalg.norm(C-canonical)*1000)
        pose_pass=bool(not stop and samples==4050 and supported/4050>.8 and error<5 and tilt<5 and center_error<2 and drift<3 and shift<2)
        certificate=SweepCertificate(model,check.hand,.3);lower,bad=certificate.query(data,C,B);final_bad=[model.geom(int(certificate.ids[i])).name for i in bad];certified=0;minimum=None
        if pose_pass and not bad and lower>.3:
            geometry_data=restore(model,z);minimum=lower
            for qpos in history:
                geometry_data.qpos[:]=qpos;mujoco.mj_forward(model,geometry_data)
                value,bad=certificate.query(geometry_data,geometry_data.geom_xpos[cube],geometry_data.geom_xmat[cube].reshape(3,3));minimum=min(minimum,value);certified+=1
                if bad or value<=.3:break
        passed=bool(pose_pass and certified==4050 and minimum>.3)
        trial=folder/name;trial.mkdir();np.savez_compressed(trial/'research_goals.npz',start=start,goals=commands)
        np.savez_compressed(trial/'end_state.npz',**{key:getattr(data,key) for key in BOUND},time=data.time,
            up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        row=dict(name=name,path=trial.relative_to(OUT).as_posix(),steps=len(commands),stop=stop,phases=phase_results,
            final_orientation_error_deg=error,final_tilt_deg=tilt,canonical_station_error_mm=center_error,hold_samples=samples,
            whole_hand_support_fraction=supported/max(1,samples),hold_angle_drift_deg=drift,hold_center_drift_mm=shift,
            final_upper_lower_mm=lower,final_bad_upper_geoms=final_bad,certified_hold_samples=certified,minimum_upper_certificate_lower_mm=minimum,
            actual_checkpoint_to_work_grip_passed=passed,worst=check.worst,terminal_guard=check.last,terminal_support=sample_model(model,data,check.hand,cube))
        save_json(trial/'review.json',row);rows.append(row);print(json.dumps({key:row[key] for key in ('name','steps','stop','final_orientation_error_deg','canonical_station_error_mm','actual_checkpoint_to_work_grip_passed')}),flush=True)
    save_json(folder/'review.json',dict(status='MF_RF_escape_waypoint_work_grip_finishing',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),
        reference=reference_path.relative_to(OUT).as_posix(),reference_sha256=sha(reference_path),model_provenance=provenance,rows=rows,
        actual_state_resets_within_each_trial=0,original_guards_geometry_and_joint_limits_preserved=True,
        whole_initial_placement_chain_not_replayed_by_this_study=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
