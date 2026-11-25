"""Conditional model robustness of the newly connected clear working grip.

The constant external moments are hypotheses, not measured left-hand loads.
No physical motor torque/current conversion or deployment is performed.
"""
from pathlib import Path
import argparse
import json
import time
import numpy as np
import mujoco
from native_cycle import OUT, assets, restore, Guard, BOUND, save_json, sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target, orientation_error_deg, face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--source-state');parser.add_argument('--tag',default='clear_work_grip_load_hypotheses')
    parser.add_argument('--station-scope',choices=('canonical','source'),default='canonical');args=parser.parse_args()
    if not args.tag.replace('_','').isalnum():raise ValueError('Local research tag required')
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/args.tag
    if folder.exists():raise RuntimeError('Preserve previous load study')
    source=base/'RF_clear_work_grip_joined_suffix/end_state.npz'
    if args.source_state:
        source=(OUT/args.source_state).resolve()
        if not source.is_relative_to(OUT.resolve()):raise ValueError('Local saved source required')
    z=dict(np.load(source,allow_pickle=False))
    cases=[('nominal',.3,0.),('friction025',.25,0.),('friction035',.35,0.)]
    cases += [(f'moment{moment:+g}',.3,moment) for moment in (-.0005,.0005,-.001,.001,-.003,.003,-.01,.01)]
    folder.mkdir();(folder/'source_probe_clear_work_grip_loads.py').write_bytes(Path(__file__).read_bytes());rows=[];began=time.perf_counter()
    for label,friction,moment in cases:
        model,initial,_,_,provenance=assets('active_ccd50');guard=Guard(model);cube=model.geom('cube').id;cube_body=int(model.geom_bodyid[cube])
        # Both members get the same sliding value; torsional/rolling entries remain unchanged.
        original_friction=model.geom_friction.copy()
        touched=[geom for geom in range(model.ngeom) if int(model.geom_bodyid[geom]) in guard.hand or geom==cube]
        model.geom_friction[touched,0]=friction
        data=restore(model,z);data.xfrc_applied[cube_body,3:]=initial['up']*moment
        target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90)
        start_B=data.geom_xmat[cube].reshape(3,3).copy();start_C=data.geom_xpos[cube].copy()
        station=np.load(OUT/'baseline_archived_zero/end_state.npz',allow_pickle=False)['qpos'][-7:-4]
        canonical_station=station.copy()
        if args.station_scope=='source':station=start_C.copy()
        stop=[];history=[];supported=0;angle_drift=position_drift=0.;normal_min=float('inf')
        for step in range(4050):
            mujoco.mj_step(model,data);stop=guard.check(data)
            if np.linalg.norm(data.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            support=sample_model(model,data,guard.hand,cube);supported+=support['supported_sample'];normal_min=min(normal_min,support['upward_support_N'])
            angle_drift=max(angle_drift,orientation_error_deg(data.geom_xmat[cube].reshape(3,3),start_B));position_drift=max(position_drift,float(np.linalg.norm(data.geom_xpos[cube]-start_C)*1000))
            history.append(data.qpos.copy())
            if stop:break
        B=data.geom_xmat[cube].reshape(3,3);C=data.geom_xpos[cube];error=orientation_error_deg(B,target)
        station_error=float(np.linalg.norm(C-station)*1000);tilt=face_up_error_deg(B,5,initial['up'])
        pose_pass=bool(not stop and len(history)==4050 and supported/4050>.8 and angle_drift<3 and position_drift<2 and error<5 and tilt<5 and station_error<2)
        certificate=SweepCertificate(model,guard.hand,.3);final_lower,bad=certificate.query(data,C,B)
        final_bad=[model.geom(int(certificate.ids[i])).name for i in bad];certified=0;minimum=None
        if pose_pass and not bad and final_lower>.3:
            geometry_data=restore(model,z);minimum=final_lower
            for qpos in history:
                geometry_data.qpos[:]=qpos;mujoco.mj_forward(model,geometry_data)
                lower,bad=certificate.query(geometry_data,geometry_data.geom_xpos[cube],geometry_data.geom_xmat[cube].reshape(3,3));minimum=min(minimum,lower);certified+=1
                if bad or lower<=.3:break
        passed=bool(pose_pass and certified==4050 and minimum>.3)
        trial=folder/label;trial.mkdir();np.savez_compressed(trial/'end_state.npz',**{key:getattr(data,key) for key in BOUND},time=data.time,
            up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        row=dict(case=label,assumed_sliding_friction=friction,assumed_constant_moment_about_up_Nm=moment,
            samples=len(history),stop=stop,whole_hand_supported_fraction=supported/len(history),minimum_upward_support_N=normal_min,
            angle_drift_deg=angle_drift,center_drift_mm=position_drift,final_global90_error_deg=error,common_station_error_mm=station_error,
            station_scope=args.station_scope,original_canonical_station_error_mm=float(np.linalg.norm(C-canonical_station)*1000),
            pose_and_support_passed=pose_pass,certified_window_samples=certified,upper_minimum_lower_mm=minimum,
            final_upper_certificate_lower_mm=final_lower,final_bad_upper_geoms=final_bad,conditional_hold_passed=passed,
            terminal_support=sample_model(model,data,guard.hand,cube),worst=guard.worst,
            only_sliding_friction_entry_changed=bool(np.array_equal(model.geom_friction[:,1:],original_friction[:,1:])),
            original_guards_and_joint_limits_preserved=True)
        save_json(trial/'review.json',row);rows.append(row);save_json(folder/'progress.json',dict(rows=rows))
        print(json.dumps({key:row[key] for key in ('case','samples','stop','conditional_hold_passed','angle_drift_deg','final_bad_upper_geoms')}),flush=True)
    save_json(folder/'review.json',dict(status='clear_work_grip_conditional_friction_and_load_hypotheses',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),
        model_provenance=provenance,cases=rows,source_was_reached_only_from_initialized_RF_endpoint=args.source_state is None,
        alternative_source_explicit=args.source_state is not None,station_scope=args.station_scope,original_canonical_failures_not_reclassified=True,
        applied_moment_is_hypothesis_not_measured_left_hand_load=True,model_torque_is_not_calibrated_to_450mA=True,
        first_rotation_or_next_rotation_not_evaluated=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False,
        elapsed_seconds=time.perf_counter()-began))


if __name__=='__main__':main()
