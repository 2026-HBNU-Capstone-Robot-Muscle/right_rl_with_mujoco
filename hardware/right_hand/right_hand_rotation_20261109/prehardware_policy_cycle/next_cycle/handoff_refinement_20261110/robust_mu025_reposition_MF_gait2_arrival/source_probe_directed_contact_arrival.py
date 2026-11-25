"""Free-object arrival at declared positive-yaw contacts; all hand support counts."""
from pathlib import Path
import argparse
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,SIM_TO_MOTOR_IDS,save_json,sha,angle
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate
from directed_contact_state import tip_yaw_contact


def main():
    p=argparse.ArgumentParser();p.add_argument('--geometry',required=True);p.add_argument('--tag',required=True);p.add_argument('--sliding-friction',type=float);p.add_argument('--stop-at-target',action='store_true');a=p.parse_args();geometry=(OUT/a.geometry).resolve()
    if not geometry.is_relative_to(OUT.resolve()) or not a.tag.replace('_','').isalnum():raise ValueError('Local geometry and tag required')
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/a.tag
    if folder.exists():raise RuntimeError('Preserve prior directed arrival trials')
    prior=json.loads(geometry.read_text());source=OUT/prior['source'];assert sha(source)==prior['source_sha256'];z=dict(np.load(source,allow_pickle=False));model,initial,_,_,provenance=assets('active_ccd50');cube=model.geom('cube').id;sid=np.asarray(SIM_TO_MOTOR_IDS)-1
    if a.sliding_friction is not None:
        if not .2<=a.sliding_friction<=.4:raise ValueError('Bounded offline friction hypothesis required')
        model.geom_friction[:,0]=a.sliding_friction
    first_station=np.load(base/'reached_first90_reverse_quarter_probe/first90_space_hold_state.npz',allow_pickle=False)['qpos'][-7:-4];targetB=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],180.)
    folder.mkdir();(folder/'source_probe_directed_contact_arrival.py').write_bytes(Path(__file__).read_bytes());rows=[]
    for row in prior['rows']:
        if not row['static_contact_candidate']:continue
        f=row['finger'];offset=4*('if','mf','rf','th').index(f);active=np.arange(offset,offset+4);tip=model.geom(f+'_tip').id;proposal=OUT/row['proposal'];assert sha(proposal)==row['proposal_sha256'];ik=restore(model,dict(np.load(proposal,allow_pickle=False)));ik.qvel[:]=0;mujoco.mj_forward(model,ik)
        contacts=[c for c in ik.contact if {int(c.geom1),int(c.geom2)}=={cube,tip}]
        if not contacts:continue
        c=min(contacts,key=lambda c:c.dist);other=int(c.geom2) if int(c.geom1)==cube else int(c.geom1);n=c.frame[:3]*(1 if int(c.geom1)==cube else -1);J=np.zeros((3,model.nv));mujoco.mj_jac(model,ik,J,None,c.pos,int(model.geom_bodyid[other]));closing=-J[:,:16].T@n/model.actuator_gainprm[:,0]
        balanced=ik.qpos[:16]+((ik.qfrc_bias-ik.qfrc_passive)[:16]/model.actuator_gainprm[:,0])
        for preload in (0.,.3,.6):
            label=row['profile']+f'_force{preload:g}';sub=folder/label;sub.mkdir();d=restore(model,z);g=Guard(model);C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy();desired=z['ctrl'].copy();desired[active]=balanced[active]+closing[active]*preload;target=motor(desired);start=motor(d.ctrl);previous=start.copy()
            if np.any(target<g.lo) or np.any(target>g.hi):
                result=dict(profile=label,skipped='original_goal_bounds',partial_yaw_advance_and_support_passed=False,full_next90_and_hold_passed=False);rows.append(result);save_json(sub/'review.json',result);continue
            count=max(1,int(np.ceil(np.max(abs(target-start))/2)));planned_count=count;early_goal_count=None;commands=[];trajectory=[d.qpos.copy()];stop=[];samples=supported=certified=0;holdC=holdB=None;shift=drift=0.;minimum=float('inf');cert=SweepCertificate(model,g.hand,.3);evaluate_space=False;certificate_stopped_after_failure=False;step=0
            while step<count+54:
                B=d.geom_xmat[cube].reshape(3,3)
                if a.stop_at_target and step<count and orientation_error_deg(B,targetB)<3 and face_up_error_deg(B,5,initial['up'])<5:
                    count=step;target=previous.copy();early_goal_count=step
                command=previous.copy()
                if count:command[sid[active]]=np.rint(start[sid[active]]+(target[sid[active]]-start[sid[active]])*min(step+1,count)/count)
                assert np.max(abs(command-previous))<=2+1e-9 and np.all(command>=g.lo) and np.all(command<=g.hi);d.ctrl[active]=ticks_to_sim(command)[active];fixed=np.setdiff1d(np.arange(16),active);assert np.array_equal(d.ctrl[fixed],z['ctrl'][fixed]);commands.append(command.copy());previous=command.copy()
                if step==count:evaluate_space=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),targetB)<10
                for _ in range(75):
                    mujoco.mj_step(model,d);stop=g.check(d)
                    if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                    if step>=count:
                        C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
                        if holdC is None:holdC=C.copy();holdB=B.copy()
                        samples+=1;supported+=sample_model(model,d,g.hand,cube)['supported_sample'];shift=max(shift,float(np.linalg.norm(C-holdC)*1000));drift=max(drift,orientation_error_deg(B,holdB))
                        if evaluate_space:
                            lower,bad=cert.query(d,C,B);minimum=min(minimum,lower);certified+=not bad and lower>.3
                            if a.stop_at_target and (bad or lower<=.3):evaluate_space=False;certificate_stopped_after_failure=True
                    if stop:break
                trajectory.append(d.qpos.copy())
                if stop:break
                step+=1
            yaw=angle(d.geom_xmat[cube].reshape(3,3),B0,initial['up']);error=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),targetB);center=float(np.linalg.norm(d.geom_xpos[cube]-first_station)*1000);tilt=face_up_error_deg(d.geom_xmat[cube].reshape(3,3),5,initial['up'])
            partial=bool(not stop and samples==4050 and supported/4050>.8 and yaw>5)
            reached_full=bool(not stop and samples==certified==4050 and supported/4050>.8 and error<5 and tilt<5 and shift<2 and drift<3)
            full=bool(reached_full and center<2);actual_direction=tip_yaw_contact(model,d,cube,tip,initial['up'])
            np.savez_compressed(sub/'research_goals.npz',start=start,goals=commands);np.savez_compressed(sub/'trajectory.npz',qpos=trajectory,control_seconds=.075);np.savez_compressed(sub/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
            result=dict(profile=label,path=sub.relative_to(OUT).as_posix(),finger=f,preload_hypothesis_N=preload,steps=len(commands),stop=stop,hold_samples=samples,whole_hand_support_fraction=supported/max(1,samples),yaw_advance_deg=yaw,final180_error_deg=error,station_A_error_mm=center,tilt_deg=tilt,hold_shift_mm=shift,hold_drift_deg=drift,upper_certified_samples=certified,upper_minimum_lower_mm=minimum if np.isfinite(minimum) else None,partial_yaw_advance_and_support_passed=partial,full_next90_and_hold_passed=full,rotation_and_clear_hold_at_reached_location_passed=reached_full,actual_contact_direction=actual_direction,planned_arrival_goals=planned_count,early_target_hold_after_goal=early_goal_count,upper_certificate_stopped_after_first_failure=certificate_stopped_after_failure,worst=g.worst,terminal_guard=g.last)
            rows.append(result);save_json(sub/'review.json',result);print(json.dumps({k:v for k,v in result.items() if k not in ('worst','terminal_guard')}),flush=True)
    save_json(folder/'review.json',dict(status='declared_positive_yaw_contact_free_arrivals',geometry=geometry.relative_to(OUT).as_posix(),geometry_sha256=sha(geometry),source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),model_provenance=provenance,sliding_friction_hypothesis=a.sliding_friction,stop_and_hold_at_actual_orientation_target=a.stop_at_target,rows=rows,all_actual_cube_states_free=True,whole_hand_support_includes_palm=True,specific_contact_not_required_to_persist=True,original_guards_and_bounds_preserved=True,preload_not_calibrated_to450mA=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
