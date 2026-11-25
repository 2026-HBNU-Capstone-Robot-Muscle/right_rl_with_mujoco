"""Connect the low-friction teacher to clear hold and the existing next turn.

Each comparison begins at the original perturbed placement and never resets
actual dynamic state after that. Preserves failures and original native guards.
"""
from pathlib import Path
import argparse
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    p=argparse.ArgumentParser();p.add_argument('--resume-teacher',action='store_true');p.add_argument('--finish-v4',action='store_true');args=p.parse_args()
    base=OUT/'next_cycle/handoff_refinement_20261110';root=base/'robust_cycle_preparation_20261111';teacher=base/('robust_mu025_finish_teacher_v4' if args.finish_v4 else ('robust_mu025_finish_teacher_v3' if args.resume_teacher else 'robust_mu025_transport_teacher_v2'));folder=root/('low_friction_teacher_cycle_connection_v4' if args.finish_v4 else ('low_friction_teacher_cycle_connection_v3' if args.resume_teacher else 'low_friction_teacher_cycle_connection'))
    if folder.exists():raise RuntimeError('Preserve low-friction connection trials')
    teacher_review=json.loads((teacher/'review.json').read_text());assert teacher_review['sliding_friction_hypothesis']==.25 and teacher_review['original_native_guards_and_joint_limits_preserved']
    nominal=base/'initial_second_turn_hold_replay';ref_goals=np.load(nominal/'research_goals.npz',allow_pickle=False)['goals'];teach_goals=np.load(teacher/'research_goals.npz',allow_pickle=False)['goals'];expected_teacher=np.load(teacher/'end_state.npz',allow_pickle=False)
    targets=[base/'initial_to_reconfigured_first90/end_state.npz',base/'initial_to_reached_workstation90/end_state.npz'];folder.mkdir();(folder/'source_replay_robust_teacher_to_cycle.py').write_bytes(Path(__file__).read_bytes());results=[]
    for label,route in (('keep_teacher_pose',[]),('direct_reconfigured',[targets[0]]),('open_then_reconfigured',[targets[1],targets[0]])):
        sub=folder/label;sub.mkdir();m,initial,_,_,provenance=assets('active_ccd50');m.geom_friction[:,0]=.25;d=restore(m,initial);g=Guard(m);cube=m.geom('cube').id;commands=[];trajectory=[d.qpos.copy()];stop=[];seams=[];holds=[];previous=motor(d.ctrl)
        def execute(command):
            nonlocal previous,stop
            assert np.max(abs(command-previous))<=2+1e-9 and np.all(command>=g.lo) and np.all(command<=g.hi)
            d.ctrl[:]=ticks_to_sim(command);previous=command.copy();commands.append(command.copy())
            for _ in range(75):
                mujoco.mj_step(m,d);stop=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if stop:break
            trajectory.append(d.qpos.copy())
        parts=[('original_prefix1700',ref_goals[:1700])]
        if args.resume_teacher or args.finish_v4:parts.append(('transport_teacher400',np.load(base/'robust_mu025_transport_teacher_v2/research_goals.npz',allow_pickle=False)['goals']))
        if args.finish_v4:parts.append(('finish_teacher180',np.load(base/'robust_mu025_finish_teacher_v3/research_goals.npz',allow_pickle=False)['goals']))
        parts.append(('low_friction_teacher',teach_goals))
        for stage,goals in parts:
            for command in goals:
                execute(command)
                if stop:break
            if stop:break
            if stage=='low_friction_teacher':
                errors={k:float(np.max(abs(getattr(d,k)-expected_teacher[k]))) if expected_teacher[k].size else 0. for k in BOUND};assert max(errors.values())==0.,errors;seams.append(dict(stage=stage,after_goal=len(commands),bound_state_errors=errors))
        def hold(yaw):
            nonlocal stop,previous
            cert=SweepCertificate(m,g.hand,.3);C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy();count=support=clear=0;minimum=float('inf');shift=drift=0.
            for _ in range(54):
                commands.append(previous.copy());d.ctrl[:]=ticks_to_sim(previous)
                for _ in range(75):
                    mujoco.mj_step(m,d);stop=g.check(d)
                    if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                    C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3);count+=1;support+=sample_model(m,d,g.hand,cube)['supported_sample'];lower,bad=cert.query(d,C,B);minimum=min(minimum,lower);clear+=not bad and lower>.3;shift=max(shift,float(np.linalg.norm(C-C0)*1000));drift=max(drift,orientation_error_deg(B,B0))
                    if stop:break
                trajectory.append(d.qpos.copy())
                if stop:break
            B=d.geom_xmat[cube].reshape(3,3);error=orientation_error_deg(B,upright_yaw_target(initial['initial_cube_rotation'],initial['up'],yaw));tilt=face_up_error_deg(B,5,initial['up']);passed=bool(not stop and count==clear==4050 and support/4050>.8 and shift<2 and drift<3 and error<5 and tilt<5)
            h=dict(target_yaw_deg=yaw,samples=count,supported_samples=support,upper_certified_samples=clear,minimum_upper_lower_mm=minimum,shift_mm=shift,drift_deg=drift,orientation_error_deg=error,tilt_deg=tilt,passed=passed);holds.append(h);return passed
        if not stop:
            for target in route:
                desired=motor(np.load(target,allow_pickle=False)['ctrl']);start=previous.copy();n=max(1,int(np.ceil(np.max(abs(desired-start))/2)))
                for i in range(n):
                    execute(np.rint(start+(desired-start)*(i+1)/n))
                    if stop:break
                if stop:break
        first=False;second=False
        if not stop:first=hold(90.)
        if first:
            desired=motor(np.load(targets[0],allow_pickle=False)['ctrl']);start=previous.copy();n=max(1,int(np.ceil(np.max(abs(desired-start))/2)))
            for i in range(n):
                execute(np.rint(start+(desired-start)*(i+1)/n))
                if stop:break
            for command in ref_goals[3405:-54]:
                if stop:break
                execute(command)
                if stop:break
            if not stop:second=hold(180.)
        np.savez_compressed(sub/'research_goals.npz',start=motor(initial['ctrl']),goals=commands);np.savez_compressed(sub/'trajectory.npz',qpos=trajectory,control_seconds=.075);np.savez_compressed(sub/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time)
        result=dict(path=sub.relative_to(OUT).as_posix(),case=label,steps=len(commands),stop=stop,first_hold_passed=first,second_hold_passed=second,two_turn_reached_location_holds_passed=bool(first and second),seams=seams,holds=holds,actual_state_resets_after_initial_placement=0,worst=g.worst,terminal_guard=g.last);results.append(result);save_json(sub/'review.json',result);print(json.dumps({k:v for k,v in result.items() if k not in ('terminal_guard','worst')}),flush=True)
    source_files=[teacher/'review.json',teacher/'research_goals.npz',teacher/'end_state.npz',nominal/'research_goals.npz',*targets]
    if args.resume_teacher or args.finish_v4:source_files += [base/'robust_mu025_transport_teacher_v2/review.json',base/'robust_mu025_transport_teacher_v2/research_goals.npz',base/'robust_mu025_transport_teacher_v2/end_state.npz']
    if args.finish_v4:source_files += [base/'robust_mu025_finish_teacher_v3/review.json',base/'robust_mu025_finish_teacher_v3/research_goals.npz',base/'robust_mu025_finish_teacher_v3/end_state.npz']
    save_json(folder/'review.json',dict(status='low_friction_teacher_clear_hold_and_next_turn_connections',source_files_sha256={p.relative_to(OUT).as_posix():sha(p) for p in source_files},model_provenance=provenance,sliding_friction_hypothesis=.25,rows=results,old_nominal_and_failed_perturbation_results_preserved=True,source_guards_and_joint_limits_preserved=True,physical_registration_unverified=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
