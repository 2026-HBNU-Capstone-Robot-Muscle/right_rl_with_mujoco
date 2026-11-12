"""Try previously certified open-top hand postures after the second turn.

Every actual cube state is free. Rotation/clear hold at the reached location is
reported separately from the unchanged first-station2mm task. No finger is
required to retain contact. This uses model state and is not SAC or hardware.
"""
from pathlib import Path
import argparse
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,SIM_TO_MOTOR_IDS,save_json,sha,angle
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    p=argparse.ArgumentParser();p.add_argument('--tag',default='actual_A_second_turn_clear_hold');p.add_argument('--source-state');p.add_argument('--sliding-friction',type=float);a=p.parse_args()
    if not a.tag.replace('_','').isalnum():raise ValueError('Local hold study required')
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'actual_A_second_turn_clear_hold';source=base/'actual_A_next_two_contact_centered_teacher/end_state.npz'
    folder=base/a.tag
    if a.source_state:
        source=(OUT/a.source_state).resolve()
        if not source.is_relative_to(OUT.resolve()):raise ValueError('Local actual source required')
    parent_path=source.parent/('rotation_arrival_review.json' if source.name=='rotation_arrival_state.npz' else 'review.json');parent=json.loads(parent_path.read_text())
    if folder.exists():raise RuntimeError('Preserve previous second-turn holds')
    z=dict(np.load(source,allow_pickle=False));ref_file=base/'initial_to_reconfigured_first90/end_state.npz';reference=dict(np.load(ref_file,allow_pickle=False))
    model,initial,_,_,provenance=assets('active_ccd50');cube=model.geom('cube').id;sid=np.asarray(SIM_TO_MOTOR_IDS)-1;target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],180.)
    if a.sliding_friction is not None:
        if not .2<=a.sliding_friction<=.4:raise ValueError('Bounded friction hypothesis required')
        model.geom_friction[:,0]=a.sliding_friction
    ref=restore(model,reference);station=ref.geom_xpos[cube].copy();Bfirst=ref.geom_xmat[cube].reshape(3,3).copy()
    folder.mkdir();(folder/'source_probe_second_turn_clear_hold.py').write_bytes(Path(__file__).read_bytes());rows=[]
    for label,active in (('unchanged',np.array([],dtype=int)),('MF_clear',np.arange(4,8)),('MF_TH_clear',np.r_[4:8,12:16]),('known_clear_hand',np.arange(16))):
        sub=folder/label;sub.mkdir();d=restore(model,z);g=Guard(model);start=motor(d.ctrl);previous=start.copy();desired=z['ctrl'].copy();desired[active]=reference['ctrl'][active];end=motor(desired)
        count=int(np.ceil(np.max(abs(end-start))/2));commands=[];trajectory=[d.qpos.copy()];stop=[];samples=supported=certified=0;minimum=float('inf');C0=B0=None;shift=drift=0.;cert=SweepCertificate(model,g.hand,.3)
        for step in range(count+54):
            command=previous.copy()
            if count:command[sid[active]]=np.rint(start[sid[active]]+(end[sid[active]]-start[sid[active]])*min(step+1,count)/count)
            assert np.max(abs(command-previous))<=2+1e-9 and np.all(command>=g.lo) and np.all(command<=g.hi)
            d.ctrl[active]=ticks_to_sim(command)[active];fixed=np.setdiff1d(np.arange(16),active);assert np.array_equal(d.ctrl[fixed],z['ctrl'][fixed]);commands.append(command.copy());previous=command.copy()
            for _ in range(75):
                mujoco.mj_step(model,d);stop=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if step>=count:
                    C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
                    if C0 is None:C0=C.copy();B0=B.copy()
                    samples+=1;supported+=sample_model(model,d,g.hand,cube)['supported_sample'];lower,bad=cert.query(d,C,B);minimum=min(minimum,lower);certified+=not bad and lower>.3
                    shift=max(shift,float(np.linalg.norm(C-C0)*1000));drift=max(drift,orientation_error_deg(B,B0))
                if stop:break
            trajectory.append(d.qpos.copy())
            if stop:break
        B=d.geom_xmat[cube].reshape(3,3);error=orientation_error_deg(B,target);center=float(np.linalg.norm(d.geom_xpos[cube]-station)*1000);tilt=face_up_error_deg(B,5,initial['up'])
        rotation_clear=bool(not stop and samples==certified==4050 and supported/4050>.8 and error<5 and tilt<5 and shift<2 and drift<3)
        np.savez_compressed(sub/'research_goals.npz',start=start,goals=commands);np.savez_compressed(sub/'trajectory.npz',qpos=trajectory,control_seconds=.075);np.savez_compressed(sub/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        r=dict(path=sub.relative_to(OUT).as_posix(),label=label,changed_sim_indices=active.tolist(),steps=len(commands),stop=stop,hold_samples=samples,upper_certified_samples=certified,upper_minimum_lower_mm=minimum if np.isfinite(minimum) else None,whole_hand_hold_support_fraction=supported/max(1,samples),hold_shift_mm=shift,hold_drift_deg=drift,final180_error_deg=error,additional_yaw_from_first_hold_deg=angle(B,Bfirst,initial['up']),tilt_deg=tilt,first_station_error_mm=center,first_station_2mm_passed=center<2,rotation_and_clear_hold_at_reached_location_passed=rotation_clear,same_first_station_next_hold_passed=bool(rotation_clear and center<2),worst=g.worst,terminal_guard=g.last)
        rows.append(r);save_json(sub/'review.json',r);print(json.dumps({k:v for k,v in r.items() if k not in ('worst','terminal_guard')}),flush=True)
    save_json(folder/'review.json',dict(status='second_turn_free_object_clear_hold_comparisons',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),parent_review_path=parent_path.relative_to(OUT).as_posix(),parent_review_sha256=sha(parent_path),parent_native_stopped=parent.get('stop'),reference=ref_file.relative_to(OUT).as_posix(),reference_sha256=sha(ref_file),model_provenance=provenance,sliding_friction_hypothesis=a.sliding_friction,rows=rows,whole_hand_support_includes_palm=True,contact_identity_not_required=True,all_actual_cube_states_free=True,first_station_target_and2mm_threshold_unchanged=True,original_guards_and_bounds_preserved=True,independent_initial_replay_not_yet_performed=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
