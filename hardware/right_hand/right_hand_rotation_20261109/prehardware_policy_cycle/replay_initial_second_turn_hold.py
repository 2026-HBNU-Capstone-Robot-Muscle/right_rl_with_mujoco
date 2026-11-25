"""Independently replay both turns and selected hold from original placement.

Different reached positions and unchanged2mm station requirements are reported
separately. No result here permits hardware execution or claims robustness.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha,angle
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'initial_second_turn_hold_replay'
    if folder.exists():raise RuntimeError('Preserve independent two-turn hold replay')
    hold_file=base/'actual_A_second_turn_clear_hold/review.json';holds=json.loads(hold_file.read_text())
    candidates=[r for r in holds['rows'] if not r['stop'] and r['hold_samples']==4050]
    if not candidates:raise RuntimeError('No complete native hold to replay; failures remain in the source report')
    chosen=min(candidates,key=lambda r:(not r['same_first_station_next_hold_passed'],not r['rotation_and_clear_hold_at_reached_location_passed'],-r['upper_certified_samples'],r['final180_error_deg']))
    suffix=OUT/chosen['path'];prior=base/'initial_through_next_MF_contact_gait';thumb=base/'actual_A_next_counter_thumb_arrival/axis0_sign+1_force0.6';teacher=base/'actual_A_next_two_contact_centered_teacher'
    parts=[('initial_and_partial_next4673',prior),('thumb_arrival',thumb),('centered_second_turn',teacher),('selected_final_hold',suffix)]
    model,initial,_,_,provenance=assets('active_ccd50');d=restore(model,initial);g=Guard(model);cube=model.geom('cube').id
    first_ref=restore(model,dict(np.load(base/'initial_to_reconfigured_first90/end_state.npz',allow_pickle=False)));station=first_ref.geom_xpos[cube].copy();Bfirst=first_ref.geom_xmat[cube].reshape(3,3).copy();target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],180.)
    folder.mkdir();(folder/'source_replay_initial_second_turn_hold.py').write_bytes(Path(__file__).read_bytes());commands=[];previous=motor(d.ctrl);trajectory=[d.qpos.copy()];seams=[];sources=[hold_file];samples=supported=0;holds_stats=[]
    for label,parent in parts:
        gp=parent/'research_goals.npz';ep=parent/'end_state.npz';goals=np.load(gp,allow_pickle=False)['goals'];expected=np.load(ep,allow_pickle=False);sources.extend([gp,ep]);cert=SweepCertificate(model,g.hand,.3);count=held=clear=0;minimum=float('inf');C0=B0=None;shift=drift=0.
        for index,command in enumerate(goals):
            assert np.max(abs(command-previous))<=2+1e-9 and np.all(command>=g.lo) and np.all(command<=g.hi)
            d.ctrl[:]=ticks_to_sim(command);commands.append(command.copy());previous=command.copy()
            is_first_hold=label=='initial_and_partial_next4673' and 3351<=index<3405
            is_final_hold=label=='selected_final_hold' and index>=len(goals)-54
            for _ in range(75):
                mujoco.mj_step(model,d);assert not g.check(d),g.last
                assert np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])<=.08
                supported_now=sample_model(model,d,g.hand,cube)['supported_sample'];samples+=1;supported+=supported_now
                if is_first_hold or is_final_hold:
                    C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
                    if C0 is None:C0=C.copy();B0=B.copy()
                    lower,bad=cert.query(d,C,B);minimum=min(minimum,lower);count+=1;held+=supported_now;clear+=not bad and lower>.3
                    shift=max(shift,float(np.linalg.norm(C-C0)*1000));drift=max(drift,orientation_error_deg(B,B0))
            trajectory.append(d.qpos.copy())
        errors={k:float(np.max(abs(getattr(d,k)-expected[k]))) if expected[k].size else 0. for k in BOUND};assert max(errors.values())==0.,(label,errors)
        seams.append(dict(segment=label,goals=len(goals),after_goal=len(commands),bound_state_errors=errors))
        if count:holds_stats.append(dict(stage=label,samples=count,supported_samples=held,upper_certified_samples=clear,minimum_upper_lower_mm=minimum,hold_shift_mm=shift,hold_drift_deg=drift))
        print(json.dumps(dict(segment=label,goals=len(commands),bound_state_errors=errors)),flush=True)
    assert len(holds_stats)==2 and all(r['samples']==4050 for r in holds_stats)
    B=d.geom_xmat[cube].reshape(3,3);error=orientation_error_deg(B,target);tilt=face_up_error_deg(B,5,initial['up']);center=float(np.linalg.norm(d.geom_xpos[cube]-station)*1000)
    dual_hold=bool(all(r['upper_certified_samples']==4050 and r['supported_samples']/4050>.8 and r['minimum_upper_lower_mm']>.3 and r['hold_shift_mm']<2 and r['hold_drift_deg']<3 for r in holds_stats) and error<5 and tilt<5)
    first_report=json.loads((base/'initial_to_reconfigured_first90/review.json').read_text());dual_hold=bool(dual_hold and first_report['first90_reconfigured_whole_hand_hold_passed'])
    np.savez_compressed(folder/'research_goals.npz',start=motor(initial['ctrl']),goals=commands);np.savez_compressed(folder/'trajectory.npz',qpos=trajectory,control_seconds=.075);np.savez_compressed(folder/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
    report=dict(status='initial_second_turn_and_hold_independently_replayed',source_files_sha256={p.relative_to(OUT).as_posix():sha(p) for p in sources},model_provenance=provenance,selected_hold_profile=chosen['label'],steps=len(commands),seams=seams,holds=holds_stats,actual_state_resets_after_initial_placement=0,all1ms_samples=samples,whole_hand_supported_samples=supported,whole_hand_supported_fraction=supported/samples,final180_error_deg=error,final_tilt_deg=tilt,additional_yaw_from_first_hold_deg=angle(B,Bfirst,initial['up']),first_station_error_mm=center,first_station_2mm_passed=center<2,two_rotations_and_clear_holds_at_reached_locations_passed=dual_hold,same_first_station_two_turn_cycle_passed=bool(dual_hold and center<2),old_canonical_failures_not_reclassified=True,whole_hand_support_includes_palm=True,specific_finger_contact_not_required=True,native_guards_and_joint_bounds_unchanged=True,perturbation_robustness_not_yet_evaluated=True,not_a_hardware_sensor_controller=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False,worst=g.worst)
    save_json(folder/'review.json',report);print(json.dumps({k:report[k] for k in ('steps','final180_error_deg','first_station_error_mm','two_rotations_and_clear_holds_at_reached_locations_passed','same_first_station_two_turn_cycle_passed')}),flush=True)


if __name__=='__main__':main()
