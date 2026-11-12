"""Free-physics arrival trials at a rechecked RF hold, with explicit joint orders."""
from pathlib import Path
import argparse
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,motor,ticks_to_sim,BOUND,SIM_TO_MOTOR_IDS,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--reference');parser.add_argument('--tag',default='RF_reference_arrivals')
    parser.add_argument('--work-grip',action='store_true');parser.add_argument('--source-state')
    parser.add_argument('--global-yaw',type=float,default=90.);args=parser.parse_args()
    if not args.tag.replace('_','').isalnum():raise ValueError('Invalid research tag')
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/args.tag
    if folder.exists():raise RuntimeError('Preserve previous arrival study')
    m,initial,_,_,provenance=assets('active_ccd50');cube=m.geom('cube').id;guard=Guard(m)
    pool_path=base/'early_CCD50_states/state_pool.npz';pool=np.load(pool_path,allow_pickle=False)
    index=int(np.flatnonzero(pool['indices']==1700)[0]);z=dict(initial)
    z.update({key:pool[key][index].copy() for key in BOUND});z['time']=float(pool['time'][index])
    source_path=(OUT/args.source_state).resolve() if args.source_state else pool_path
    if args.source_state:
        if not source_path.is_relative_to(OUT.resolve()):raise ValueError('Source must be local research data')
        z=dict(np.load(source_path,allow_pickle=False))
    reference_path=(OUT/args.reference).resolve() if args.reference else base/'RF_endpoint_CCD50_load_screen/qualified0_end_state.npz'
    if not reference_path.resolve().is_relative_to(OUT.resolve()):raise ValueError('Reference must be a local research state')
    reference=dict(np.load(reference_path,allow_pickle=False));target_ticks=motor(reference['ctrl'])
    if np.any(target_ticks<guard.lo) or np.any(target_ticks>guard.hi):raise RuntimeError('Reference command violates original native limits')
    if not 0<=args.global_yaw<=180:raise ValueError('Bounded global yaw required')
    orientation_target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],args.global_yaw)
    canonical_station=np.load(OUT/'baseline_archived_zero/end_state.npz',allow_pickle=False)['qpos'][-7:-4]
    all_ids=np.arange(16);rf=np.array(SIM_TO_MOTOR_IDS)[8:12]-1
    thumb_index=np.array(SIM_TO_MOTOR_IDS)[np.r_[0:4,12:16]]-1
    thumb_ring=np.array(SIM_TO_MOTOR_IDS)[8:16]-1
    orders=[('simultaneous',[all_ids]),('RF_first',[rf,np.setdiff1d(all_ids,rf)]),
            ('thumb_index_first',[thumb_index,rf,np.setdiff1d(all_ids,np.r_[thumb_index,rf])]),
            ('thumb_RF_first',[thumb_ring,np.setdiff1d(all_ids,thumb_ring)])]
    folder.mkdir();rows=[]
    for name,groups in orders:
        d=restore(m,z);check=Guard(m);previous=motor(d.ctrl);goals=[];stop=[];samples=supported=loaded=0
        for group in groups:
            start=previous.copy();end=start.copy();end[group]=target_ticks[group]
            count=max(1,int(np.ceil(np.max(abs(end-start))/2)))
            for step in range(count):
                goal=np.rint(start+(end-start)*(step+1)/count).astype(int)
                if np.max(abs(goal-previous))>2+1e-9:raise AssertionError('Rate changed')
                if np.any(goal<check.lo) or np.any(goal>check.hi):raise AssertionError('Range changed')
                d.ctrl[:]=ticks_to_sim(goal);goals.append(goal.copy());previous=goal
                for _ in range(75):
                    mujoco.mj_step(m,d);stop=check.check(d)
                    if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                    if stop:break
                if stop:break
            if stop:break
        B0=d.geom_xmat[cube].reshape(3,3).copy();C0=d.geom_xpos[cube].copy();angle_drift=center_drift=0.;history=[]
        if not stop:
            for _ in range(54):
                goals.append(previous.copy())
                for _ in range(75):
                    mujoco.mj_step(m,d);stop=check.check(d)
                    if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                    sample=sample_model(m,d,check.hand,cube);samples+=1;supported+=sample['supported_sample']
                    loaded+=sum(value['normal_N'] for key,value in sample['by_geom'].items() if key.startswith('rf_'))>.2
                    angle_drift=max(angle_drift,orientation_error_deg(d.geom_xmat[cube].reshape(3,3),B0))
                    center_drift=max(center_drift,float(np.linalg.norm(d.geom_xpos[cube]-C0)*1000))
                    history.append(d.qpos.copy())
                    if stop:break
                if stop:break
        error=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),orientation_target)
        position_error=float(np.linalg.norm(d.geom_xpos[cube]-reference['qpos'][-7:-4])*1000)
        canonical_error=float(np.linalg.norm(d.geom_xpos[cube]-canonical_station)*1000)
        passed=bool(not stop and samples==4050 and supported/4050>.8 and loaded/4050>.8 and angle_drift<3 and center_drift<2 and error<5 and position_error<2)
        pose_pass=bool(not stop and samples==4050 and supported/4050>.8 and angle_drift<3 and center_drift<2 and error<5 and position_error<2 and canonical_error<2 and face_up_error_deg(d.geom_xmat[cube].reshape(3,3),5,initial['up'])<5)
        certified=0;minimum=None
        if args.work_grip and pose_pass:
            certificate=SweepCertificate(m,check.hand,.3)
            value,bad=certificate.query(d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3));minimum=value
            if not bad and value>.3:
                geometry_data=restore(m,z)
                for qpos in history:
                    geometry_data.qpos[:]=qpos;mujoco.mj_forward(m,geometry_data)
                    value,bad=certificate.query(geometry_data,geometry_data.geom_xpos[cube],geometry_data.geom_xmat[cube].reshape(3,3));minimum=min(minimum,value);certified+=1
                    if bad or value<=.3:break
        rows.append(dict(order=name,steps=len(goals),stop=stop,hold_samples=samples,
            whole_hand_supported_fraction=supported/max(1,samples),RF_loaded_fraction=loaded/max(1,samples),
            final_global90_error_deg=error,final_reference_center_error_mm=position_error,
            global_yaw_target_deg=args.global_yaw,canonical_station_error_mm=canonical_error,
            hold_angle_drift_deg=angle_drift,hold_center_drift_mm=center_drift,
            first_rotation_and_RF_hold_from_checkpoint_passed=passed,
            first_rotation_and_work_grip_from_checkpoint_passed=bool(args.work_grip and pose_pass and certified==4050 and minimum>.3),
            strict_upper_certificate_samples=certified,strict_upper_minimum_lower_mm=minimum,
            terminal_guard=check.last,worst=check.worst,
            final_upper_space_and_full_initial_placement_cycle_not_certified=True))
        np.savez_compressed(folder/(name+'_goals.npz'),start=motor(z['ctrl']),goals=goals)
        np.savez_compressed(folder/(name+'_end_state.npz'),**{key:getattr(d,key) for key in BOUND},time=d.time,
            up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        print(json.dumps({key:rows[-1][key] for key in ('order','steps','stop','final_global90_error_deg','first_rotation_and_RF_hold_from_checkpoint_passed')}),flush=True)
    report=dict(status='joint_order_arrival_trials_at_rechecked_RF_hold',model_provenance=provenance,rows=rows,
        pool_sha256=sha(pool_path),reference_sha256=sha(reference_path),cube_free_through_every_arrival=True,
        reference_path=reference_path.relative_to(OUT).as_posix(),work_grip_evaluation_requested=args.work_grip,
        source_state=source_path.relative_to(OUT).as_posix(),source_state_sha256=sha(source_path),source_override_is_assembled_state_not_original_pool=args.source_state is not None,
        no_resets_within_each_initialized_checkpoint_trial=True,whole_cycle_or_hardware_success=False,
        devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'review.json',report)
    (folder/'source_probe_RF_reference_arrivals.py').write_bytes(Path(__file__).read_bytes())


if __name__=='__main__':main()
