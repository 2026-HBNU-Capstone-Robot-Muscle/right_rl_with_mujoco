"""Reuse the original108 actor through the current bounded native governor.

This changes execution timing/rate, so successful inference is not training
parity or proof that the original task covered arbitrary yaw goals.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT, ROOT, assets, restore, Guard, BOUND, motor, ticks_to_sim, TICK, save_json, sha
from evaluate_saved_policies_current_model_20261109 import load_module, quat
from saved_policy_observation_provider_20261109 import sensor_value
from cube_task_frame_20261105 import upright_yaw_target, orientation_error_deg, face_up_error_deg
from whole_hand_support import sample_model


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'original108_governed_bridge_probes'
    if folder.exists():raise RuntimeError('Preserve previous original policy probes')
    work=ROOT.parent/'work/Leap_Test';actor_path=work/'policy/policy_np.npz';implementation=work/'deploy/policy_np.py';obs_path=work/'deploy/obs_builder.py'
    actor=load_module('bridge_original_actor',implementation).Policy(actor_path)
    builder_type=load_module('bridge_original_observations',obs_path).ObsBuilder
    if actor.mean.shape!=(108,):raise RuntimeError('Original policy input contract changed')
    model,initial,_,_,provenance=assets('active_ccd50');cube=model.geom('cube').id
    pool_path=base/'early_CCD50_states/state_pool.npz';pool=np.load(pool_path,allow_pickle=False);index=int(np.flatnonzero(pool['indices']==1700)[0])
    entry=dict(initial);entry.update({key:pool[key][index].copy() for key in BOUND});entry['time']=float(pool['time'][index])
    hold_path=base/'RF_clear_work_grip_joined_suffix/end_state.npz';hold=dict(np.load(hold_path,allow_pickle=False))
    cases=[('entry90',entry,90.),('next5',hold,95.),('next90',hold,180.)]
    folder.mkdir();(folder/'source_probe_original_policy_governed_bridge.py').write_bytes(Path(__file__).read_bytes());rows=[]
    training_low=np.array([-.314,-1.047,-.2,-.2]*3+[-.349,-.47,-1.2,-1.34])
    training_high=np.array([2.23,1.047,1.885,2.042]*3+[2.094,2.443,1.9,1.88])
    for label,z,yaw in cases:
        for last_action_semantics in ('raw_actor','executed_increment'):
            data=restore(model,z);guard=Guard(model);builder=builder_type();target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],yaw);goal=quat(target)
            builder.fill_ori_history(sensor_value(model,data,'cube_orientation',4),goal)
            start=motor(data.ctrl);goals=[];trace=[];stop=[];best=float('inf')
            for step in range(800):
                previous=data.ctrl.copy();previous_ticks=motor(previous)
                observation=builder.build(np.rint(data.qpos[:16]/TICK)*TICK,previous,
                    sensor_value(model,data,'palm_position',3),sensor_value(model,data,'cube_position',3),sensor_value(model,data,'cube_orientation',4),goal)
                action=actor(observation)
                raw_proposal=np.clip(previous+.2*action,training_low,training_high)
                proposal=np.clip(motor(raw_proposal),np.ceil(guard.lo).astype(int),np.floor(guard.hi).astype(int))
                command=previous_ticks+np.clip(proposal-previous_ticks,-2,2)
                if np.any(command<guard.lo) or np.any(command>guard.hi):raise RuntimeError('Native range changed')
                data.ctrl[:]=ticks_to_sim(command);goals.append(command.copy())
                builder.set_last_act(action if last_action_semantics=='raw_actor' else (data.ctrl-previous)/.2)
                for _ in range(75):
                    mujoco.mj_step(model,data);stop=guard.check(data)
                    if np.linalg.norm(data.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                    if stop:break
                error=orientation_error_deg(data.geom_xmat[cube].reshape(3,3),target);best=min(best,error)
                trace.append(dict(step=step+1,error_deg=error,raw_max_goal_delta_ticks=int(np.max(abs(proposal-previous_ticks))),
                    saturated_actor_dimensions=int(np.count_nonzero(abs(action)>.99)),maximum_normalized_observation=float(np.max(abs((observation-actor.mean)/actor.std))),
                    whole_hand_support=sample_model(model,data,guard.hand,cube)))
                if stop:break
            trial=folder/(label+'_'+last_action_semantics);trial.mkdir();np.savez_compressed(trial/'research_goals.npz',start=start,goals=goals)
            np.savez_compressed(trial/'end_state.npz',**{key:getattr(data,key) for key in BOUND},time=data.time,
                up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
            row=dict(case=label,last_action_semantics=last_action_semantics,global_yaw_target_deg=yaw,steps=len(goals),stop=stop,
                initial_error_deg=orientation_error_deg(restore(model,z).geom_xmat[cube].reshape(3,3),target),best_error_deg=best,final_error_deg=trace[-1]['error_deg'],
                final_tilt_deg=face_up_error_deg(data.geom_xmat[cube].reshape(3,3),5,initial['up']),trace=trace,worst=guard.worst,terminal_guard=guard.last,
                actual_state_resets_after_start=0,complete_turn_and_final_hold_not_certified=True)
            save_json(trial/'review.json',row);rows.append(dict(path=trial.relative_to(OUT).as_posix(),**row))
            print(json.dumps({key:row[key] for key in ('case','last_action_semantics','steps','stop','best_error_deg')}),flush=True)
    save_json(folder/'review.json',dict(status='original108_actor_with_current_two_tick_governor',actor_sha256=sha(actor_path),implementation_sha256=sha(implementation),observation_builder_sha256=sha(obs_path),
        model_provenance=provenance,source_pool_sha256=sha(pool_path),source_clear_work_grip_sha256=sha(hold_path),rows=rows,
        actual_joint_observations_quantized_to_encoder_ticks=True,object_pose_and_training_sensor_locations_exact_model=True,
        execution_governor_changes_original_training_timing_and_actions=True,arbitrary_yaw_task_compatibility_not_claimed=True,
        local110_SAC_reference_actor_not_used=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
