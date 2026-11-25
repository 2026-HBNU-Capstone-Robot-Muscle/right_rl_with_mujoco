"""Independent replay of actual reached segments, with explicit sample timing.

Only the original placement is initialized. A short interrupted research hold
is recorded as such; it cannot silently certify the 75ms execution schedule.
"""
from pathlib import Path
import argparse
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha,SIM_TO_MOTOR_IDS
from cube_task_frame_20261105 import orientation_error_deg,upright_yaw_target,face_up_error_deg
from whole_hand_support import sample_model
from upper_layer_separation_certificate_20261105 import SweepCertificate

def main():
    p=argparse.ArgumentParser();p.add_argument('--tag',required=True);p.add_argument('--complete-75ms',action='store_true');p.add_argument('--tail',default='robust_mu025_feedback_gait_push_v11/force0.8');p.add_argument('--append-segment',action='append',default=[]);p.add_argument('--verify-final-hold',action='store_true');p.add_argument('--sliding-friction',type=float,default=.25);p.add_argument('--initial-x-mm',type=float,default=0.);a=p.parse_args()
    if not .2<=a.sliding_friction<=.4 or not -1<=a.initial_x_mm<=1:raise ValueError('Bounded declared finite perturbation required')
    if not a.tag.replace('_','').isalnum():raise ValueError('Local tag required')
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/a.tag
    tail=(base/a.tail).resolve()
    if folder.exists() or not tail.is_relative_to(base.resolve()):raise ValueError('New local replay required')
    first=base/'robust_cycle_preparation_20261111/low_friction_teacher_cycle_connection_v4/keep_teacher_pose'
    firststate=base/'robust_cycle_preparation_20261111/low_friction_first_hold_state_v4/end_state.npz'
    parts=[('first_hold',first,firststate,2478,None),
        ('position_first',base/'robust_mu025_reposition_to_nominal_entry_v1',None,None,None),
        ('ordered_regrasp',base/'reposition_then_second_rotation_20261111/finger_alignment_orders_v1/RF_IF_MF_TH',None,None,None),
        ('first_orientation_repair',base/'robust_mu025_entry_orientation_repair_v3',None,None,None),
        ('MF_first_arrival',base/'robust_mu025_reposition_MF_contact_arrival_v1/axis0_sign+1_force0',None,None,slice(4,8)),
        ('MF_first_reset',base/'robust_mu025_reposition_MF_reset_gait_v1/pip-30_dip15_mcp10',None,None,slice(4,8)),
        ('MF_first_free_approach',base/'robust_mu025_free_MF_standoff_route_v1',None,None,slice(4,8)),
        ('MF_first_feedback',base/'robust_mu025_declared_face_feedback_v4/force0.5',None,None,slice(4,8)),
        ('MF_abduction_reset',base/'robust_mu025_feedback_reset_v7/pip-15_dip0_mcp0',None,None,slice(4,8)),
        ('MF_second_free_approach',base/'robust_mu025_feedback_free_route_v10',None,None,slice(4,8)),
        ('MF_second_feedback',tail,None,None,slice(4,8))]
    for index,name in enumerate(a.append_segment):
        extra=(base/name).resolve()
        if not extra.is_relative_to(base.resolve()):raise ValueError('Local continuation segment required')
        saved=np.load(extra/'research_goals.npz',allow_pickle=False);changed_ids=np.flatnonzero(np.any(saved['goals']!=saved['start'],axis=0))+1;changed_sim=np.flatnonzero(np.isin(np.asarray(SIM_TO_MOTOR_IDS),changed_ids))
        parts.append((f'continuation{index+1}',extra,None,None,changed_sim))
    m,initial,_,_,provenance=assets('active_ccd50');m.geom_friction[:,0]=a.sliding_friction;initial['qpos'][-7]+=a.initial_x_mm/1000;d=restore(m,initial);g=Guard(m);cube=m.geom('cube').id;previous=motor(d.ctrl);commands=[];durations=[];sources={};seams=[];trajectory=[d.qpos.copy()];stop=[];first_hold={};final_hold={};first_station=None;sample_count=supported=0
    folder.mkdir();(folder/'source_replay_low_friction_feedback_chain.py').write_bytes(Path(__file__).read_bytes())
    original_time=float(d.time)
    for segment_index,(label,parent,endpoint,count,active) in enumerate(parts):
        gp=parent/'research_goals.npz';ep=endpoint or parent/'end_state.npz';z=np.load(gp,allow_pickle=False);goals=z['goals'][:count] if count else z['goals'];expected=np.load(ep,allow_pickle=False);sources[gp.relative_to(OUT).as_posix()]=sha(gp);sources[ep.relative_to(OUT).as_posix()]=sha(ep)
        expected_samples=int(round((float(expected['time'])-original_time)/.001))
        if a.complete_75ms:expected_samples=len(goals)*75
        last_samples=expected_samples-(len(goals)-1)*75
        assert 1<=last_samples<=75,(label,expected_samples,last_samples)
        cert=SweepCertificate(m,g.hand,.3);held=clear=hcount=0;minimum=None;holdB=holdC=None;drift=shift=0.
        for i,command in enumerate(goals):
            assert np.max(abs(command-previous))<=2 and np.all(command>=g.lo) and np.all(command<=g.hi)
            q=ticks_to_sim(command)
            if active is None:d.ctrl[:]=q
            else:d.ctrl[active]=q[active]
            previous=command.copy();commands.append(command.copy());n=last_samples if i==len(goals)-1 else 75;durations.append(n)
            executed_samples=0
            for _ in range(n):
                mujoco.mj_step(m,d);stop=g.check(d);sample_count+=1
                executed_samples+=1
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                is_first=label=='first_hold' and i>=len(goals)-54
                is_final=a.verify_final_hold and segment_index==len(parts)-1 and i>=len(goals)-54
                if is_first or is_final:
                    C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
                    if holdC is None:holdC=C.copy();holdB=B.copy()
                    hcount+=1;held+=sample_model(m,d,g.hand,cube)['supported_sample'];lower,bad=cert.query(d,C,B);clear+=not bad and lower>.3;minimum=lower if minimum is None else min(minimum,lower);shift=max(shift,float(np.linalg.norm(C-holdC)*1000));drift=max(drift,orientation_error_deg(B,holdB))
                if stop:break
            durations[-1]=executed_samples
            trajectory.append(d.qpos.copy())
            if stop:break
        errors={k:float(np.max(abs(getattr(d,k)-expected[k]))) if expected[k].size else 0. for k in BOUND}
        if not a.complete_75ms and not stop:assert max(errors.values())==0.,(label,errors)
        original_time=float(expected['time'])
        seam=dict(segment=label,completed_goals=i+1,planned_goals=len(goals),after_goal=len(commands),final_command_duration_ms=n,bound_state_errors=errors,stop=stop)
        seams.append(seam)
        if hcount:
            target_yaw=90 if label=='first_hold' else 180
            result_hold=dict(samples=hcount,supported_samples=held,upper_certified_samples=clear,minimum_upper_lower_mm=minimum,shift_mm=shift,drift_deg=drift,orientation_error_deg=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),upright_yaw_target(initial['initial_cube_rotation'],initial['up'],target_yaw)),tilt_deg=face_up_error_deg(d.geom_xmat[cube].reshape(3,3),5,initial['up']))
            if label=='first_hold':first_hold=result_hold;first_station=d.geom_xpos[cube].copy()
            else:final_hold=result_hold
        save_json(folder/'progress.json',dict(seams=seams,samples=sample_count,stop=stop));print(json.dumps(seam),flush=True)
        if stop:break
    B=d.geom_xmat[cube].reshape(3,3);error=orientation_error_deg(B,upright_yaw_target(initial['initial_cube_rotation'],initial['up'],180))
    def hold_pass(r):return bool(r and r['samples']==r['upper_certified_samples']==4050 and r['supported_samples']/4050>.8 and r['minimum_upper_lower_mm']>.3 and r['shift_mm']<2 and r['drift_deg']<3 and r['orientation_error_deg']<5 and r['tilt_deg']<5)
    full_pass=bool(not stop and len(seams)==len(parts) and hold_pass(first_hold) and hold_pass(final_hold));first_station_error=float(np.linalg.norm(d.geom_xpos[cube]-first_station)*1000) if first_station is not None else None
    np.savez_compressed(folder/'research_goals.npz',start=motor(initial['ctrl']),goals=commands,duration_ms=durations);np.savez_compressed(folder/'trajectory.npz',qpos=trajectory,control_seconds=.075);np.savez_compressed(folder/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
    r=dict(status='low_friction_actual_feedback_chain_independently_replayed',source_files_sha256=sources,model_provenance=provenance,steps=len(commands),stop=stop,seams=seams,first_hold=first_hold,final_hold=final_hold,sliding_friction_hypothesis=a.sliding_friction,initial_cube_worldX_offset_mm=a.initial_x_mm,all1ms_samples=sample_count,original75ms_schedule_used=bool(all(n==75 for n in durations)),short_interrupted_research_commands=sum(n!=75 for n in durations),complete75ms_counterfactual_requested=a.complete_75ms,actual_state_resets_after_initial_placement=0,final180_error_deg=error,final_tilt_deg=face_up_error_deg(B,5,initial['up']),original_native_guards_and_joint_limits_preserved=True,full_two_turn_two_hold_success=full_pass,first_station_error_mm=first_station_error,same_first_station_2mm_cycle_passed=bool(full_pass and first_station_error<2),reached_location_hold_scope_not_left_hand_reach_or_load_validation=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False,worst=g.worst)
    save_json(folder/'review.json',r);print(json.dumps({k:v for k,v in r.items() if k not in ('source_files_sha256','seams','model_provenance')}),flush=True)

if __name__=='__main__':main()
