"""Test the existing second-turn sequence from a genuinely reached entry state.

A slightly missed first-hold task is retained rather than suppressing the next
turn diagnostic. No complete-cycle success is claimed without replaying it.
"""
from pathlib import Path
import argparse
import json
import numpy as np
import mujoco
from native_cycle import OUT, assets, restore, Guard, BOUND, motor, ticks_to_sim, save_json, sha, angle
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg, upright_yaw_target, face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--tag',required=True);p.add_argument('--align',action='store_true');p.add_argument('--reference-initialization-control',action='store_true');a=p.parse_args()
    if not a.tag.replace('_','').isalnum():raise ValueError('Local study tag required')
    source=(OUT/a.source).resolve()
    if not source.is_relative_to(OUT.resolve()):raise ValueError('Local reached state required')
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'reposition_then_second_rotation_20261111/suffix_trials'/a.tag
    if folder.exists():raise RuntimeError('Preserve prior suffix trial')
    reference=base/'initial_to_reconfigured_first90/end_state.npz';suffix=base/'initial_second_turn_hold_replay/research_goals.npz'
    if (source==reference.resolve())!=a.reference_initialization_control:raise ValueError('Reference initialization must be explicitly identified as a control experiment')
    model,initial,_,_,provenance=assets('active_ccd50');model.geom_friction[:,0]=.25;data=restore(model,dict(np.load(source,allow_pickle=False)));g=Guard(model);cube=model.geom('cube').id
    ref=restore(model,dict(np.load(reference,allow_pickle=False)));refC=ref.geom_xpos[cube].copy();refB=ref.geom_xmat[cube].reshape(3,3).copy();desired=motor(ref.ctrl)
    initialB=data.geom_xmat[cube].reshape(3,3).copy();previous=motor(data.ctrl);commands=[];trajectory=[data.qpos.copy()];stop=[]
    def metrics():
        B=data.geom_xmat[cube].reshape(3,3);return dict(cube_center_error_to_entry_mm=float(np.linalg.norm(data.geom_xpos[cube]-refC)*1000),cube_orientation_error_to_entry_deg=orientation_error_deg(B,refB),first90_error_deg=orientation_error_deg(B,upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90.)),final180_error_deg=orientation_error_deg(B,upright_yaw_target(initial['initial_cube_rotation'],initial['up'],180.)),tilt_deg=face_up_error_deg(B,5,initial['up']),command_error_to_entry_ticks=float(np.max(abs(motor(data.ctrl)-desired))))
    def execute(command,hold=False,cert=None,stats=None):
        nonlocal previous,stop
        assert np.max(abs(command-previous))<=2+1e-9 and np.all(command>=g.lo) and np.all(command<=g.hi)
        data.ctrl[:]=ticks_to_sim(command);previous=command.copy();commands.append(command.copy())
        for _ in range(75):
            mujoco.mj_step(model,data);stop=g.check(data)
            if np.linalg.norm(data.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            if hold:
                C=data.geom_xpos[cube];B=data.geom_xmat[cube].reshape(3,3);stats['samples']+=1;stats['supported']+=sample_model(model,data,g.hand,cube)['supported_sample'];stats['shift_mm']=max(stats['shift_mm'],float(np.linalg.norm(C-stats['C0'])*1000));stats['drift_deg']=max(stats['drift_deg'],orientation_error_deg(B,stats['B0']))
                if cert is not None:
                    lower,bad=cert.query(data,C,B);stats['certified']+=not bad and lower>.3;stats['minimum_lower_mm']=min(stats['minimum_lower_mm'],lower)
            if stop:break
        trajectory.append(data.qpos.copy())
    before=metrics();aligned=None
    if a.align:
        start=previous.copy();n=max(1,int(np.ceil(np.max(abs(desired-start))/2)))
        for i in range(n):
            execute(np.rint(start+(desired-start)*(i+1)/n))
            if stop:break
        aligned=metrics()
    if not stop and np.max(abs(previous-desired))>2:raise ValueError('Existing suffix requires actual aligned command; use --align')
    alignment_steps=len(commands)
    if not stop:
        for command in np.load(suffix,allow_pickle=False)['goals'][3405:-54]:
            execute(command)
            if stop:break
    before_hold=metrics();stats=None;upper_precheck=None
    if not stop:
        cert=SweepCertificate(model,g.hand,.3);lower,bad=cert.query(data,data.geom_xpos[cube],data.geom_xmat[cube].reshape(3,3));upper_precheck=dict(lower_mm=lower,uncertified_geoms=[model.geom(int(cert.ids[i])).name for i in bad])
        full_certificate=before_hold['final180_error_deg']<5 and before_hold['tilt_deg']<5 and not bad and lower>.3
        stats=dict(samples=0,supported=0,certified=0,minimum_lower_mm=None,C0=data.geom_xpos[cube].copy(),B0=data.geom_xmat[cube].reshape(3,3).copy(),shift_mm=0.,drift_deg=0.)
        if full_certificate:stats['minimum_lower_mm']=float('inf')
        for _ in range(54):
            execute(previous,hold=True,cert=cert if full_certificate else None,stats=stats)
            if stop:break
        stats.pop('C0');stats.pop('B0')
    final=metrics();passed=bool(not stop and stats and stats['samples']==stats['certified']==4050 and stats['supported']/4050>.8 and stats['shift_mm']<2 and stats['drift_deg']<3 and final['final180_error_deg']<5 and final['tilt_deg']<5)
    folder.mkdir(parents=True);(folder/'source_replay_second_suffix_from_reached_entry.py').write_bytes(Path(__file__).read_bytes())
    np.savez_compressed(folder/'research_goals.npz',start=motor(np.load(source,allow_pickle=False)['ctrl']),goals=commands);np.savez_compressed(folder/'trajectory.npz',qpos=trajectory,control_seconds=.075)
    np.savez_compressed(folder/'end_state.npz',**{k:getattr(data,k) for k in BOUND},time=data.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
    result=dict(status='existing_second_turn_from_reached_entry_diagnostic',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),reference=reference.relative_to(OUT).as_posix(),reference_sha256=sha(reference),suffix=suffix.relative_to(OUT).as_posix(),suffix_sha256=sha(suffix),model_provenance=provenance,sliding_friction_hypothesis=.25,
        steps=len(commands),alignment_steps=alignment_steps,stop=stop,before=before,after_alignment=aligned,before_hold=before_hold,final=final,hold=stats,upper_precheck=upper_precheck,second_rotation_and_clear_hold_passed=passed,
        additional_yaw_from_trial_start_deg=angle(data.geom_xmat[cube].reshape(3,3),initialB,initial['up']),actual_state_resets_after_reached_entry=0,initialized_reference_control_not_low_friction_arrival=a.reference_initialization_control,original_first_hold_failure_not_reclassified=True,full_initial_placement_two_turn_cycle_passed=False,original_native_guards_and_joint_limits_preserved=True,worst=g.worst,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'review.json',result);print(json.dumps(dict(steps=len(commands),stop=stop,before=before,after_alignment=aligned,final=final,passed=passed)),flush=True)


if __name__=='__main__':main()
