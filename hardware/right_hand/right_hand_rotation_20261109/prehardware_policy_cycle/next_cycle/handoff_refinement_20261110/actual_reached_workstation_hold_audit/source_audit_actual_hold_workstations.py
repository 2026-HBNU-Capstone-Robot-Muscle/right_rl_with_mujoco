"""Separate actual support/upper-space holds from the old fixed station metric.

This does not relax or reclassify the existing canonical task. It asks whether
the reached states already satisfy support and upper clearance at their own
locations, and reports the position differences for later hand integration.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'actual_reached_workstation_hold_audit'
    if folder.exists():raise RuntimeError('Preserve prior held-station evidence')
    sources=[('first90_RF_park',90.,base/'first90_RF_shape_before_adduction/RF_park_MF_then_RF_shape_then_adduct/phase0_end_state.npz'),
        ('first90_MF_open_RF_park',90.,base/'first90_RF_shape_before_adduction/RF_park_MF_then_RF_shape_then_adduct/phase1_end_state.npz'),
        ('next180_checkpoint600',180.,base/'next90_recenter_checkpoint/state.npz')]
    folder.mkdir();(folder/'source_audit_actual_hold_workstations.py').write_bytes(Path(__file__).read_bytes());rows=[]
    canonical=np.load(OUT/'baseline_archived_zero/end_state.npz',allow_pickle=False)['qpos'][-7:-4]
    for label,yaw,path in sources:
        model,initial,_,_,provenance=assets('active_ccd50');z=dict(np.load(path,allow_pickle=False));d=restore(model,z);guard=Guard(model);cube=model.geom('cube').id
        C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy();target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],yaw)
        certificate=SweepCertificate(model,guard.hand,.3);lower,bad=certificate.query(d,C0,B0);initial_bad=[model.geom(int(certificate.ids[i])).name for i in bad]
        stop=guard.check(d);count=certified=supported=0;shift=drift=0.;minimum=lower
        if not stop and not bad and lower>.3:
            for index in range(4050):
                mujoco.mj_step(model,d);stop=guard.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                value,failed=certificate.query(d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3));minimum=min(minimum,value);certified+=not failed and value>.3
                count+=1;supported+=sample_model(model,d,guard.hand,cube)['supported_sample'];shift=max(shift,float(np.linalg.norm(d.geom_xpos[cube]-C0)*1000));drift=max(drift,orientation_error_deg(d.geom_xmat[cube].reshape(3,3),B0))
                if stop or failed or value<=.3:break
        error=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),target);tilt=face_up_error_deg(d.geom_xmat[cube].reshape(3,3),5,initial['up'])
        row=dict(label=label,source=path.relative_to(OUT).as_posix(),source_sha256=sha(path),model_provenance=provenance,goal_yaw=yaw,initial_upper_bad_geoms=initial_bad,
            initial_upper_lower_mm=lower,hold_samples=count,whole_hand_support_fraction=supported/max(1,count),upper_certified_samples=certified,minimum_upper_lower_mm=minimum,
            maximum_hold_shift_mm=shift,maximum_hold_drift_deg=drift,final_orientation_error_deg=error,tilt_deg=tilt,canonical_station_error_mm=float(np.linalg.norm(d.geom_xpos[cube]-canonical)*1000),
            reached_station_world_m=C0.tolist(),stop=stop,support_space_pose_hold_at_reached_station_passed=bool(not stop and count==certified==4050 and supported/4050>.8 and shift<2 and drift<3 and error<5 and tilt<5),
            old_canonical_task_not_reclassified=True,left_hand_reach_and_load_at_this_station_not_validated=True,terminal_support=sample_model(model,d,guard.hand,cube),worst=guard.worst)
        sub=folder/label;sub.mkdir();np.savez_compressed(sub/'end_state.npz',**{key:getattr(d,key) for key in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        rows.append(row);save_json(sub/'review.json',row);print(json.dumps({key:row[key] for key in ('label','initial_upper_bad_geoms','hold_samples','final_orientation_error_deg','canonical_station_error_mm','support_space_pose_hold_at_reached_station_passed')}),flush=True)
    save_json(folder/'review.json',dict(status='separate_reached_station_hold_diagnostic',rows=rows,original_canonical_and_common_two_turn_task_not_reclassified=True,
        free_object_held_from_actual_saved_states=True,no_new_cold_initialization_of_cube=True,entire_first_and_next_rotation_chain_not_replayed_by_this_diagnostic=True,
        full_two_turn_cycle_passed=False,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
