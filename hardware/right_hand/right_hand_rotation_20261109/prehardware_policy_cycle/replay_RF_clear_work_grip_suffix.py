"""Join the IF and RF clearance stages without resetting their actual state."""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT, assets, restore, Guard, BOUND, motor, ticks_to_sim, sha, save_json
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target, orientation_error_deg, face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'RF_clear_work_grip_joined_suffix'
    if folder.exists():raise RuntimeError('Preserve previous joined replay')
    first_path=base/'RF_index_clearance_margin2/review.json';first=json.loads(first_path.read_text())
    second_path=base/'RF_clearance_free_arrivals/review.json';second=json.loads(second_path.read_text())
    selected=next(row for row in second['trials'] if row['initialized_clear_work_grip_suffix_passed'])
    first_trial=OUT/first['dynamic_trials'][0]['path'];second_trial=OUT/selected['path']
    source=OUT/first['source']
    if sha(source)!=first['source_sha256'] or sha(first_trial/'end_state.npz')!=second['source_sha256']:raise RuntimeError('Stage provenance changed')
    one=dict(np.load(first_trial/'research_goals.npz',allow_pickle=False));two=dict(np.load(second_trial/'research_goals.npz',allow_pickle=False))
    if not np.array_equal(one['goals'][-1],two['start']):raise RuntimeError('Command seam changed')
    model,initial,_,_,provenance=assets('active_ccd50');z=dict(np.load(source,allow_pickle=False));data=restore(model,z);guard=Guard(model);cube=model.geom('cube').id
    commands=np.vstack([one['goals'],two['goals']]);start=motor(data.ctrl)
    if not np.array_equal(start,one['start']) or np.max(abs(np.diff(np.vstack([start,commands]),axis=0)))>2:raise RuntimeError('Rate/start contract changed')
    folder.mkdir();(folder/'source_replay_RF_clear_work_grip_suffix.py').write_bytes(Path(__file__).read_bytes())
    stop=[];trace=[data.qpos.copy()];seams=[];samples=supported=certified=0;minimum=float('inf');certificate=SweepCertificate(model,guard.hand,.3)
    hold_B=hold_C=None;angle_drift=position_drift=0.
    for step,command in enumerate(commands):
        if np.any(command<guard.lo) or np.any(command>guard.hi):raise RuntimeError('Native command range changed')
        data.ctrl[:]=ticks_to_sim(command)
        for _ in range(75):
            mujoco.mj_step(model,data);stop=guard.check(data)
            if np.linalg.norm(data.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            if step>=len(commands)-54:
                samples+=1;supported+=sample_model(model,data,guard.hand,cube)['supported_sample']
                C=data.geom_xpos[cube];B=data.geom_xmat[cube].reshape(3,3)
                lower,bad=certificate.query(data,C,B);minimum=min(minimum,lower);certified+=not bad and lower>.3
                if hold_B is None:hold_B=B.copy();hold_C=C.copy()
                angle_drift=max(angle_drift,orientation_error_deg(B,hold_B));position_drift=max(position_drift,float(np.linalg.norm(C-hold_C)*1000))
            if stop:break
        trace.append(data.qpos.copy())
        if step+1 in (len(one['goals']),len(commands)):
            target_state=dict(np.load((first_trial if step+1==len(one['goals']) else second_trial)/'end_state.npz',allow_pickle=False))
            differences={key:float(np.max(abs(getattr(data,key)-target_state[key]))) if target_state[key].size else 0. for key in BOUND}
            seams.append(dict(after_goals=step+1,state_max_abs_difference=differences,exact_bound_state_equal=all(value==0 for value in differences.values())))
        if stop:break
    target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90)
    station=np.load(OUT/'baseline_archived_zero/end_state.npz',allow_pickle=False)['qpos'][-7:-4]
    error=orientation_error_deg(data.geom_xmat[cube].reshape(3,3),target);tilt=face_up_error_deg(data.geom_xmat[cube].reshape(3,3),5,initial['up'])
    center_error=float(np.linalg.norm(data.geom_xpos[cube]-station)*1000)
    passed=bool(not stop and samples==4050 and certified==4050 and supported/4050>.8 and minimum>.3 and error<5 and tilt<5 and center_error<2 and angle_drift<3 and position_drift<2)
    np.savez_compressed(folder/'research_goals.npz',start=start,goals=commands[:len(trace)-1])
    np.savez_compressed(folder/'trajectory.npz',qpos=trace,control_seconds=.075,index_clearance_end=len(one['goals']))
    np.savez_compressed(folder/'end_state.npz',**{key:getattr(data,key) for key in BOUND},time=data.time,
        up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
    report=dict(status='conditional_RF_endpoint_to_clear_work_grip_joined_replay',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),model_provenance=provenance,
        source_reports_sha256={path.relative_to(OUT).as_posix():sha(path) for path in (first_path,second_path)},
        steps=len(trace)-1,stage_goals=[len(one['goals']),len(two['goals'])],stop=stop,seams=seams,
        actual_state_resets_after_start=0,final_hold_samples=samples,certified_samples=certified,
        upper_certificate_minimum_lower_mm=minimum if samples else None,whole_hand_support_fraction=supported/max(1,samples),
        hold_angle_drift_deg=angle_drift,hold_center_drift_mm=position_drift,final_global90_error_deg=error,final_tilt_deg=tilt,common_station_error_mm=center_error,
        conditional_clear_work_grip_suffix_passed=passed,terminal_support=sample_model(model,data,guard.hand,cube),worst=guard.worst,
        initial_RF_endpoint_itself_was_initialized_not_reached_from_initial_placement=True,
        first_rotation_from_initial_placement_or_next_rotation_not_certified=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'review.json',report)
    print(json.dumps(dict(steps=report['steps'],passed=passed,seams=seams,gap_mm=report['upper_certificate_minimum_lower_mm'])),flush=True)


if __name__=='__main__':main()
