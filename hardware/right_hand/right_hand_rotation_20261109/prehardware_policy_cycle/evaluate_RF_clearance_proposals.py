"""Test RF withdrawal proposals with free cube support and full sweep checks.

RF contact is measured separately. Whole-hand support does not require an
arbitrary minimum at each individual finger. No geometry or guard is removed.
"""
from pathlib import Path
from collections import deque
import json
import time
import numpy as np
import mujoco
from native_cycle import OUT, assets, restore, Guard, BOUND, motor, ticks_to_sim, save_json, sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target, orientation_error_deg, face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'RF_clearance_free_arrivals'
    if folder.exists():raise RuntimeError('Preserve previous clearance arrivals')
    study_path=base/'RF_clear_global_after_index/review.json';study=json.loads(study_path.read_text())
    source=OUT/study['source']
    if sha(source)!=study['source_sha256']:raise RuntimeError('Source state changed')
    model,initial,_,_,provenance=assets('active_ccd50');cube=model.geom('cube').id
    z=dict(np.load(source,allow_pickle=False));start_data=restore(model,z);start=motor(start_data.ctrl)
    target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90)
    station=np.load(OUT/'baseline_archived_zero/end_state.npz',allow_pickle=False)['qpos'][-7:-4]
    folder.mkdir();(folder/'source_evaluate_RF_clearance_proposals.py').write_bytes(Path(__file__).read_bytes())
    began=time.perf_counter();rows=[]
    for proposal in study['proposals']:
        proposal_path=OUT/proposal['proposal']
        if sha(proposal_path)!=proposal['proposal_sha256']:raise RuntimeError('Proposal changed')
        goal=motor(np.load(proposal_path,allow_pickle=False)['ctrl']);data=restore(model,z);guard=Guard(model)
        count=max(1,int(np.ceil(np.max(abs(goal-start))/2)));previous=start.copy();goals=[];stop=[];history=deque(maxlen=4050)
        samples=supported=rf_contact=rf_loaded=0;normal_min=float('inf');normal_max=0.;angle_drift=position_drift=0.;hold_B=hold_C=None
        for step in range(count+134):
            command=np.rint(start+(goal-start)*min(step+1,count)/count).astype(int)
            if np.max(abs(command-previous))>2 or np.any(command<guard.lo) or np.any(command>guard.hi):raise AssertionError('Command contract changed')
            data.ctrl[:]=ticks_to_sim(command);previous=command;goals.append(command.copy())
            for _ in range(75):
                mujoco.mj_step(model,data);stop=guard.check(data)
                if np.linalg.norm(data.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if step>=count+80:
                    sample=sample_model(model,data,guard.hand,cube);samples+=1;supported+=sample['supported_sample']
                    normal=sum(value['normal_N'] for name,value in sample['by_geom'].items()
                        if (model.body(int(model.geom_bodyid[model.geom(name).id])).name or '').startswith('rf_'))
                    normal_min=min(normal_min,normal);normal_max=max(normal_max,normal);rf_contact+=normal>1e-5;rf_loaded+=normal>.2
                    C=data.geom_xpos[cube];B=data.geom_xmat[cube].reshape(3,3)
                    if hold_B is None:hold_B=B.copy();hold_C=C.copy()
                    angle_drift=max(angle_drift,orientation_error_deg(B,hold_B));position_drift=max(position_drift,float(np.linalg.norm(C-hold_C)*1000));history.append(data.qpos.copy())
                if stop:break
            if stop:break
        B=data.geom_xmat[cube].reshape(3,3);C=data.geom_xpos[cube]
        error=orientation_error_deg(B,target);tilt=face_up_error_deg(B,5,initial['up']);station_error=float(np.linalg.norm(C-station)*1000)
        pose_pass=bool(not stop and samples==4050 and supported/4050>.8 and angle_drift<3 and position_drift<2 and error<5 and tilt<5 and station_error<2)
        certified=0;minimum=None;bad=[];certificate=SweepCertificate(model,guard.hand,.3)
        # Query final pose first, then certify only the recorded final window.
        lower,bad=certificate.query(data,C,B);final_lower=lower;final_bad=[model.geom(int(certificate.ids[i])).name for i in bad]
        if pose_pass and not bad and lower>.3:
            geometry_data=restore(model,z);minimum=lower
            for qpos in history:
                geometry_data.qpos[:]=qpos;mujoco.mj_forward(model,geometry_data)
                lower,bad=certificate.query(geometry_data,geometry_data.geom_xpos[cube],geometry_data.geom_xmat[cube].reshape(3,3))
                minimum=min(minimum,lower);certified+=1
                if bad or lower<=.3:break
        passed=bool(pose_pass and certified==4050 and not bad and minimum>.3)
        trial=folder/proposal['geom'];trial.mkdir();np.savez_compressed(trial/'research_goals.npz',start=start,goals=goals)
        np.savez_compressed(trial/'end_state.npz',**{key:getattr(data,key) for key in BOUND},time=data.time,
            up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        row=dict(path=trial.relative_to(OUT).as_posix(),proposal=proposal['proposal'],proposal_sha256=proposal['proposal_sha256'],steps=len(goals),stop=stop,
            hold_samples=samples,whole_hand_support_fraction=supported/max(1,samples),RF_contact_fraction=rf_contact/max(1,samples),RF_over_point2N_fraction=rf_loaded/max(1,samples),
            RF_normal_min_N=normal_min if samples else None,RF_normal_max_N=normal_max if samples else None,
            hold_angle_drift_deg=angle_drift,hold_center_drift_mm=position_drift,final_global90_error_deg=error,final_tilt_deg=tilt,common_station_error_mm=station_error,
            pose_and_whole_hand_support_passed=pose_pass,certified_window_samples=certified,upper_certificate_minimum_lower_mm=minimum,
            final_upper_certificate_lower_mm=final_lower,final_bad_upper_geoms=final_bad,
            initialized_clear_work_grip_suffix_passed=passed,RF_third_point_contact_passed=bool(passed and rf_contact/4050>.8),
            worst=guard.worst,terminal_guard=guard.last,terminal_support=sample_model(model,data,guard.hand,cube))
        save_json(trial/'review.json',row);rows.append(row);print(json.dumps(dict(trial=proposal['geom'],steps=len(goals),stop=stop,passed=passed,error_deg=error,lower_mm=minimum,RF_fraction=row['RF_contact_fraction'])),flush=True)
    report=dict(status='RF_clearance_proposal_free_arrivals',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),
        proposal_study=study_path.relative_to(OUT).as_posix(),proposal_study_sha256=sha(study_path),model_provenance=provenance,
        trials=rows,initialized_clear_work_grip_suffix_passed=any(row['initialized_clear_work_grip_suffix_passed'] for row in rows),
        source_is_conditional_initialized_RF_endpoint_not_reached_from_initial_placement=True,
        full_initial_placement_two_turn_cycle_passed=False,other_hand_load_not_evaluated=True,original_native_guards_and_limits_preserved=True,
        RF_force_measured_separately_from_whole_hand_support=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False,
        elapsed_seconds=time.perf_counter()-began)
    save_json(folder/'review.json',report)


if __name__=='__main__':main()
