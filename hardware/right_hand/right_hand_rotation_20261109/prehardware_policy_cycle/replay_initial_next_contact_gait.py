"""Replay the initial first turn and subsequent supported MF contact gait.

This is a partial second quarter, not a full-cycle or hardware approval.
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
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'initial_through_next_MF_contact_gait'
    if folder.exists():raise RuntimeError('Preserve prior full-prefix contact-gait replay')
    first=base/'initial_to_reconfigured_first90';checkpoint=base/'actual_A_next_checkpoint376';reset=base/'actual_A_next_MF_reset376/pip-30_dip15_mcp10';arrival=base/'actual_A_next_MF_positive_yaw_arrival/axis2_sign-1_force0.6';pressure=base/'actual_A_next_MF_yaw_pressure/force0.5'
    parts=[('first90_reconfigured3405',first/'research_goals.npz',first/'end_state.npz'),('next_partial376',checkpoint/'goals.npz',checkpoint/'state.npz'),('MF_reposition201',reset/'research_goals.npz',reset/'end_state.npz'),('positive_contact_arrival191',arrival/'research_goals.npz',arrival/'end_state.npz'),('MF_pressure500',pressure/'research_goals.npz',pressure/'end_state.npz')]
    model,initial,_,_,provenance=assets('active_ccd50');d=restore(model,initial);g=Guard(model);cube=model.geom('cube').id;first_ref=restore(model,dict(np.load(first/'end_state.npz',allow_pickle=False)));Bnext=first_ref.geom_xmat[cube].reshape(3,3).copy();Cfirst=first_ref.geom_xpos[cube].copy();target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],180.)
    folder.mkdir();(folder/'source_replay_initial_next_contact_gait.py').write_bytes(Path(__file__).read_bytes());commands=[];trajectory=[d.qpos.copy()];previous=motor(d.ctrl);seams=[];sources=[];samples=supported=0;cert=SweepCertificate(model,g.hand,.3);first_hold=first_certified=0;first_min=float('inf')
    for name,goalsfile,endfile in parts:
        goals=np.load(goalsfile,allow_pickle=False)['goals'];sources.extend([goalsfile,endfile]);expected=np.load(endfile,allow_pickle=False)
        for index,command in enumerate(goals):
            assert np.max(abs(command-previous))<=2+1e-9 and np.all(command>=g.lo) and np.all(command<=g.hi)
            d.ctrl[:]=ticks_to_sim(command);previous=command.copy();commands.append(command.copy())
            for _ in range(75):
                mujoco.mj_step(model,d);assert not g.check(d),g.last
                assert np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])<=.08
                samples+=1;supported+=sample_model(model,d,g.hand,cube)['supported_sample']
                if name=='first90_reconfigured3405' and index>=len(goals)-54:
                    first_hold+=1;lower,bad=cert.query(d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3));first_certified+=not bad and lower>.3;first_min=min(first_min,lower)
            trajectory.append(d.qpos.copy())
        errors={k:float(np.max(abs(getattr(d,k)-expected[k]))) if expected[k].size else 0. for k in BOUND};assert max(errors.values())==0.,(name,errors)
        seams.append(dict(segment=name,goals=len(goals),after_goal=len(commands),bound_state_errors=errors));print(json.dumps(dict(segment=name,after_goal=len(commands),exact_bound_match=True)),flush=True)
    assert len(commands)==4673 and first_hold==first_certified==4050
    final_cert=SweepCertificate(model,g.hand,.3);lower,bad=final_cert.query(d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3))
    np.savez_compressed(folder/'research_goals.npz',start=motor(initial['ctrl']),goals=commands);np.savez_compressed(folder/'trajectory.npz',qpos=trajectory,control_seconds=.075);np.savez_compressed(folder/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
    report=dict(status='initial_through_partial_second_turn_contact_gait_replayed',source_files_sha256={p.relative_to(OUT).as_posix():sha(p) for p in sources},model_provenance=provenance,steps=len(commands),seams=seams,actual_state_resets_after_initial_placement=0,
        all1ms_samples=samples,whole_hand_supported_samples=supported,whole_hand_support_fraction=supported/samples,first_hold_samples=first_hold,first_hold_upper_certified_samples=first_certified,first_hold_upper_minimum_lower_mm=first_min,
        additional_next_yaw_deg=angle(d.geom_xmat[cube].reshape(3,3),Bnext,initial['up']),final180_error_deg=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),target),reference_first_station_error_mm=float(np.linalg.norm(d.geom_xpos[cube]-Cfirst)*1000),
        final_pose_upper_lower_mm=lower,final_pose_upper_bad_geoms=[model.geom(int(final_cert.ids[i])).name for i in bad],final_pose_certificate_is_not_a_full_hold=True,
        specific_finger_contact_not_required=True,whole_hand_support_includes_palm=True,partial_second_turn_not_full90=True,full_initial_two_positive_turn_cycle_passed=False,original_guards_and_bounds_preserved=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False,worst=g.worst)
    save_json(folder/'review.json',report);print(json.dumps({k:report[k] for k in ('steps','additional_next_yaw_deg','final180_error_deg','reference_first_station_error_mm','whole_hand_support_fraction')}),flush=True)


if __name__=='__main__':main()
