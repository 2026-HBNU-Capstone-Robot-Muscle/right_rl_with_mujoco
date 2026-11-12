"""Replay conditional clear-grip and next-turn stages without actual resets."""
from pathlib import Path
import argparse
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha,angle
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--include-next90',action='store_true');args=parser.parse_args()
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/('clear_grip_next90_joined_chain' if args.include_next90 else 'clear_grip_next5_joined_chain')
    if folder.exists():raise RuntimeError('Preserve previous joined bridge')
    paths=[base/'RF_clear_work_grip_joined_suffix',base/'horizon_fast_next5_from_clear_grip']
    if args.include_next90:paths.append(base/'horizon_next90_after_next5')
    reports=[json.loads((path/'review.json').read_text()) for path in paths]
    source=OUT/reports[0]['source'];z=dict(np.load(source,allow_pickle=False))
    if sha(source)!=reports[0]['source_sha256']:raise RuntimeError('Initial conditional source changed')
    for index in range(1,len(paths)):
        if sha(paths[index-1]/'end_state.npz')!=reports[index]['source_sha256']:raise RuntimeError('Source seam provenance changed')
    model,initial,_,_,provenance=assets('active_ccd50');data=restore(model,z);guard=Guard(model);cube=model.geom('cube').id
    start=motor(data.ctrl);commands=[];seams=[];trace=[data.qpos.copy()];boundaries=[];snapshots=[]
    for path in paths:
        saved=dict(np.load(path/'research_goals.npz',allow_pickle=False))
        if not np.array_equal(saved['start'],commands[-1] if commands else start):raise RuntimeError('Command seam changed')
        commands.extend(saved['goals']);boundaries.append(len(commands));snapshots.append(dict(np.load(path/'end_state.npz',allow_pickle=False)))
    commands=np.asarray(commands)
    if np.max(abs(np.diff(np.vstack([start,commands]),axis=0)))>2:raise RuntimeError('Two tick contract changed')
    folder.mkdir();(folder/'source_replay_clear_bridge_chain.py').write_bytes(Path(__file__).read_bytes())
    reference_center=np.load(paths[0]/'end_state.npz',allow_pickle=False)['qpos'][-7:-4]
    canonical=np.load(OUT/'baseline_archived_zero/end_state.npz',allow_pickle=False)['qpos'][-7:-4]
    target_yaw=180. if args.include_next90 else 95.;tolerance=5. if args.include_next90 else 1.
    target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],target_yaw)
    certificate=SweepCertificate(model,guard.hand,.3);samples=supported=certified=0;minimum=float('inf');stop=[]
    hold_B=hold_C=None;drift=shift=0.;start_next_B=None
    for step,command in enumerate(commands):
        if np.any(command<guard.lo) or np.any(command>guard.hi):raise RuntimeError('Native goal range changed')
        data.ctrl[:]=ticks_to_sim(command)
        for _ in range(75):
            mujoco.mj_step(model,data);stop=guard.check(data)
            if np.linalg.norm(data.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            if step>=len(commands)-54:
                C=data.geom_xpos[cube];B=data.geom_xmat[cube].reshape(3,3)
                if hold_B is None:hold_B=B.copy();hold_C=C.copy()
                samples+=1;supported+=sample_model(model,data,guard.hand,cube)['supported_sample']
                lower,bad=certificate.query(data,C,B);minimum=min(minimum,lower);certified+=not bad and lower>.3
                drift=max(drift,orientation_error_deg(B,hold_B));shift=max(shift,float(np.linalg.norm(C-hold_C)*1000))
            if stop:break
        trace.append(data.qpos.copy())
        if step+1 in boundaries:
            index=boundaries.index(step+1);expected=snapshots[index]
            differences={key:float(np.max(abs(getattr(data,key)-expected[key]))) if expected[key].size else 0. for key in BOUND}
            seams.append(dict(after_goals=step+1,max_abs_by_state=differences,all_bound_state_exact=all(value==0 for value in differences.values())))
            if index==0:start_next_B=data.geom_xmat[cube].reshape(3,3).copy()
        if stop:break
    B=data.geom_xmat[cube].reshape(3,3);C=data.geom_xpos[cube]
    error=orientation_error_deg(B,target);tilt=face_up_error_deg(B,5,initial['up'])
    ref_error=float(np.linalg.norm(C-reference_center)*1000);canonical_error=float(np.linalg.norm(C-canonical)*1000)
    passed=bool(not stop and samples==certified==4050 and minimum>.3 and supported/4050>.8 and error<tolerance and tilt<5 and drift<3 and shift<2 and ref_error<2)
    np.savez_compressed(folder/'research_goals.npz',start=start,goals=commands[:len(trace)-1]);np.savez_compressed(folder/'trajectory.npz',qpos=trace,control_seconds=.075,boundaries=boundaries)
    np.savez_compressed(folder/'end_state.npz',**{key:getattr(data,key) for key in BOUND},time=data.time,
        up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
    report=dict(status='conditional_clear_grip_and_next_turn_joined_replay',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),model_provenance=provenance,
        stages=[dict(path=path.relative_to(OUT).as_posix(),report_sha256=sha(path/'review.json'),goals_sha256=sha(path/'research_goals.npz')) for path in paths],
        steps=len(trace)-1,boundaries=boundaries,stop=stop,seams=seams,actual_state_resets_after_start=0,
        target_global_yaw_deg=target_yaw,final_orientation_error_deg=error,final_tilt_deg=tilt,
        additional_yaw_after_clear_grip_deg=angle(B,start_next_B,initial['up']) if start_next_B is not None else None,
        reference_grip_center_error_mm=ref_error,canonical_station_error_mm=canonical_error,
        final_hold_samples=samples,certified_samples=certified,upper_certificate_min_lower_mm=minimum if samples else None,
        whole_hand_support_fraction=supported/max(1,samples),hold_angle_drift_deg=drift,hold_center_drift_mm=shift,
        conditional_reference_station_task_passed=passed,conditional_canonical_station_task_passed=bool(passed and canonical_error<2),
        next90_requested=args.include_next90,next90_passed=bool(args.include_next90 and passed and canonical_error<2),
        initial_RF_endpoint_was_initialized_not_reached_from_initial_placement=True,full_initial_placement_two_turn_cycle_passed=False,
        model_based_teacher_not_SAC=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False,worst=guard.worst)
    save_json(folder/'review.json',report);print(json.dumps({key:report[key] for key in ('steps','final_orientation_error_deg','additional_yaw_after_clear_grip_deg','canonical_station_error_mm','conditional_reference_station_task_passed','conditional_canonical_station_task_passed','next90_passed')}),flush=True)


if __name__=='__main__':main()
