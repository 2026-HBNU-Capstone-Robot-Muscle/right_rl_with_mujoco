"""Replay the conditional MF/TH-grip next quarter between two named stations.

The start was initialized separately. The earlier first3015-goal path does not
reach this start, so a successful replay must not certify the full two-turn task.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha,angle
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'resumed_next90_joined_branch'
    if folder.exists():raise RuntimeError('Preserve earlier joined branch')
    parent=base/'opposing_workspace_next90';checkpoint=base/'thumb_branch_finish_checkpoint168';finish=base/'resumed_rotation_clear_hold/trial1'
    source=parent/'start_state.npz';z=dict(np.load(source,allow_pickle=False));model,initial,_,_,provenance=assets('active_ccd50');data=restore(model,z);guard=Guard(model);cube=model.geom('cube').id
    C0=data.geom_xpos[cube].copy();B0=data.geom_xmat[cube].reshape(3,3).copy();target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],180.)
    refpath=base/'initial_to_reached_workstation90/end_state.npz';station=np.load(refpath,allow_pickle=False)['qpos'][-7:-4]
    parts=[('quarter_first360',np.load(parent/'research_goals.npz',allow_pickle=False)['goals'],parent/'end_state.npz'),
        ('resume168',np.load(checkpoint/'goals.npz',allow_pickle=False)['goals'],checkpoint/'state.npz'),
        ('MF_clear_and_hold109',np.load(finish/'research_goals.npz',allow_pickle=False)['goals'],finish/'end_state.npz')]
    assert [len(p[1]) for p in parts]==[360,168,109]
    folder.mkdir();(folder/'source_replay_resumed_next90_branch.py').write_bytes(Path(__file__).read_bytes());commands=[];trajectory=[data.qpos.copy()];seams=[];previous=motor(data.ctrl)
    cert=SweepCertificate(model,guard.hand,.3);count=certified=supported=0;minimum=float('inf');holdC=holdB=None;shift=drift=0.
    for label,goals,endfile in parts:
        for step,goal in enumerate(goals):
            assert np.max(abs(goal-previous))<=2+1e-9 and np.all(goal>=guard.lo) and np.all(goal<=guard.hi)
            data.ctrl[:]=ticks_to_sim(goal);previous=goal.copy();commands.append(goal.copy())
            for _ in range(75):
                mujoco.mj_step(model,data);assert not guard.check(data),guard.last
                assert np.linalg.norm(data.qpos[-7:-4]-initial['initial_cube_center'])<=.08
                if label=='MF_clear_and_hold109' and step>=len(goals)-54:
                    C=data.geom_xpos[cube];B=data.geom_xmat[cube].reshape(3,3)
                    if holdC is None:holdC=C.copy();holdB=B.copy()
                    count+=1;supported+=sample_model(model,data,guard.hand,cube)['supported_sample'];value,bad=cert.query(data,C,B);minimum=min(minimum,value);certified+=not bad and value>.3
                    shift=max(shift,float(np.linalg.norm(C-holdC)*1000));drift=max(drift,orientation_error_deg(B,holdB))
            trajectory.append(data.qpos.copy())
        expected=dict(np.load(endfile,allow_pickle=False));errors={k:float(np.max(abs(getattr(data,k)-expected[k]))) if expected[k].size else 0. for k in BOUND};assert max(errors.values())==0.,(label,errors)
        seams.append(dict(segment=label,goals=len(goals),after_goal=len(commands),bound_state_errors=errors));print(json.dumps(dict(segment=label,exact_bound_state_match=True,after_goal=len(commands))),flush=True)
    error=orientation_error_deg(data.geom_xmat[cube].reshape(3,3),target);station_error=float(np.linalg.norm(data.geom_xpos[cube]-station)*1000)
    passed=bool(count==certified==4050 and supported/4050>.8 and error<5 and station_error<2 and shift<2 and drift<3)
    np.savez_compressed(folder/'research_goals.npz',start=motor(z['ctrl']),goals=commands);np.savez_compressed(folder/'trajectory.npz',qpos=trajectory,control_seconds=.075)
    np.savez_compressed(folder/'end_state.npz',**{k:getattr(data,k) for k in BOUND},time=data.time,up=initial['up'],initial_cube_rotation=initial['initial_cube_rotation'],initial_cube_center=initial['initial_cube_center'])
    sources=[source,parent/'research_goals.npz',parent/'end_state.npz',checkpoint/'goals.npz',checkpoint/'state.npz',finish/'research_goals.npz',finish/'end_state.npz',refpath]
    report=dict(status='conditional_next90_between_explicit_workstations_replayed',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),source_files_sha256={p.relative_to(OUT).as_posix():sha(p) for p in sources},model_provenance=provenance,
        steps=len(commands),seams=seams,actual_state_resets_after_start=0,additional_yaw_deg=angle(data.geom_xmat[cube].reshape(3,3),B0,initial['up']),final180_error_deg=error,
        target_station_world_m=station.tolist(),target_station_error_mm=station_error,start_station_world_m=C0.tolist(),start_to_target_station_distance_mm=float(np.linalg.norm(station-C0)*1000),
        hold_samples=count,whole_hand_support_fraction=supported/max(1,count),upper_certified_samples=certified,upper_minimum_lower_mm=minimum,hold_shift_mm=shift,hold_drift_deg=drift,
        conditional_next90_between_two_stations_passed=passed,source_is_initialized_MF_TH_grip_not_photo_RF_grip=True,source_not_reached_by_first3015_goal_path=True,
        same_station_cycle_passed=False,full_initial_placement_two_turn_cycle_passed=False,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False,worst=guard.worst)
    save_json(folder/'review.json',report);print(json.dumps({k:report[k] for k in ('steps','additional_yaw_deg','final180_error_deg','target_station_error_mm','start_to_target_station_distance_mm','upper_minimum_lower_mm','conditional_next90_between_two_stations_passed')}),flush=True)


if __name__=='__main__':main()
