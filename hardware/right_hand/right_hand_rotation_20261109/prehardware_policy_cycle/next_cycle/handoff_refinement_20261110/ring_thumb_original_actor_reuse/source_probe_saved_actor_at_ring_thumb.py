"""Read-only original actor reuse at the newly settled RF/TH model grip.

Exact model object observations are assumed. This is neither camera deployment
nor SAC training, and the source has a hypothetical initialized ancestor.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,ROOT,assets,restore,Guard,BOUND,motor,ticks_to_sim,TICK,save_json,sha
from evaluate_saved_policies_current_model_20261109 import load_module,quat
from saved_policy_observation_provider_20261109 import sensor_value
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from whole_hand_support import sample_model
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'ring_thumb_original_actor_reuse'
    if folder.exists():raise RuntimeError('Preserve previous actor comparison')
    source=base/'ring_thumb_photo_prior_seed431/axis0_ring+1/free_hold_tests/force0p8/end_state.npz';z=dict(np.load(source,allow_pickle=False))
    work=ROOT.parent/'work/Leap_Test';weights=work/'policy/policy_np.npz';implementation=work/'deploy/policy_np.py';obsfile=work/'deploy/obs_builder.py'
    actor=load_module('ring_grip_original_actor',implementation).Policy(weights);builder_class=load_module('ring_grip_original_builder',obsfile).ObsBuilder
    assert actor.mean.shape==(108,)
    model,initial,_,_,provenance=assets('active_ccd50');cube=model.geom('cube').id;target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],180.);goal=quat(target)
    training_low=np.array([-.314,-1.047,-.2,-.2]*3+[-.349,-.47,-1.2,-1.34]);training_high=np.array([2.23,1.047,1.885,2.042]*3+[2.094,2.443,1.9,1.88])
    folder.mkdir();(folder/'source_probe_saved_actor_at_ring_thumb.py').write_bytes(Path(__file__).read_bytes());rows=[]
    for semantics in ('raw_actor','executed_increment'):
        data=restore(model,z);guard=Guard(model);builder=builder_class();builder.fill_ori_history(sensor_value(model,data,'cube_orientation',4),goal)
        start=motor(data.ctrl);C0=data.geom_xpos[cube].copy();commands=[];trace=[];stop=[];best=float('inf');hold=None
        for step in range(800):
            previous=data.ctrl.copy();previous_ticks=motor(previous)
            observation=builder.build(np.rint(data.qpos[:16]/TICK)*TICK,previous,sensor_value(model,data,'palm_position',3),sensor_value(model,data,'cube_position',3),sensor_value(model,data,'cube_orientation',4),goal)
            action=actor(observation);raw=np.clip(previous+.2*action,training_low,training_high);proposal=np.clip(motor(raw),np.ceil(guard.lo),np.floor(guard.hi));command=previous_ticks+np.clip(proposal-previous_ticks,-2,2)
            assert np.all(command>=guard.lo) and np.all(command<=guard.hi) and np.max(abs(command-previous_ticks))<=2+1e-9
            data.ctrl[:]=ticks_to_sim(command);commands.append(command.copy());builder.set_last_act(action if semantics=='raw_actor' else (data.ctrl-previous)/.2)
            for _ in range(75):
                mujoco.mj_step(model,data);stop=guard.check(data)
                if np.linalg.norm(data.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if stop:break
            error=orientation_error_deg(data.geom_xmat[cube].reshape(3,3),target);best=min(best,error);shift=float(np.linalg.norm(data.geom_xpos[cube]-C0)*1000)
            trace.append(dict(step=step+1,error_deg=error,station_shift_mm=shift,support=sample_model(model,data,guard.hand,cube)))
            if stop:break
            if error<5 and shift<2 and face_up_error_deg(data.geom_xmat[cube].reshape(3,3),5,initial['up'])<5:
                cert=SweepCertificate(model,guard.hand,.3);lower,bad=cert.query(data,data.geom_xpos[cube],data.geom_xmat[cube].reshape(3,3))
                if bad or lower<=.3:continue
                supported=certified=count=0;minimum=lower
                for _ in range(54):
                    commands.append(motor(data.ctrl))
                    for _ in range(75):
                        mujoco.mj_step(model,data);stop=guard.check(data)
                        if np.linalg.norm(data.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                        value,bad=cert.query(data,data.geom_xpos[cube],data.geom_xmat[cube].reshape(3,3));minimum=min(minimum,value);certified+=not bad and value>.3;count+=1;supported+=sample_model(model,data,guard.hand,cube)['supported_sample']
                        if stop or bad or value<=.3:break
                    if stop or bad or value<=.3:break
                hold=dict(samples=count,certified_samples=certified,supported_fraction=supported/max(1,count),minimum_upper_lower_mm=minimum,
                    passed=bool(not stop and count==certified==4050 and supported/4050>.8 and orientation_error_deg(data.geom_xmat[cube].reshape(3,3),target)<5 and np.linalg.norm(data.geom_xpos[cube]-C0)<.002))
                break
        sub=folder/semantics;sub.mkdir();np.savez_compressed(sub/'research_goals.npz',start=start,goals=commands)
        np.savez_compressed(sub/'end_state.npz',**{k:getattr(data,k) for k in BOUND},time=data.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        row=dict(last_action_semantics=semantics,steps=len(commands),stop=stop,best_error_deg=best,final_error_deg=orientation_error_deg(data.geom_xmat[cube].reshape(3,3),target),
            final_station_shift_mm=float(np.linalg.norm(data.geom_xpos[cube]-C0)*1000),hold=hold,next_rotation_and_hold_passed=bool(hold and hold['passed']),trace=trace,worst=guard.worst,terminal_guard=guard.last)
        rows.append(row);save_json(sub/'review.json',row);print(json.dumps({k:row[k] for k in ('last_action_semantics','steps','stop','best_error_deg','next_rotation_and_hold_passed')}),flush=True)
    save_json(folder/'review.json',dict(status='original108_actor_at_settled_ring_thumb_grip',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),actor_sha256=sha(weights),implementation_sha256=sha(implementation),observation_builder_sha256=sha(obsfile),
        model_provenance=provenance,rows=rows,source_has_hypothetical_initialized_ancestor=True,photo_not_metrically_reconstructed=True,quantized_joint_and_exact_model_object_observations=True,
        execution_governor_changes_training_action_timing=True,arbitrary_yaw_training_compatibility_not_assumed=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
