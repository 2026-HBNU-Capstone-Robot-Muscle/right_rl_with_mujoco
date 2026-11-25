"""Run fresh encoder/pose whole-cycle proposals in perturbed native dynamics."""
from pathlib import Path
import argparse
import json
import time
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha
from shadow_runtime import SIM_IDS,TICK
from whole_cycle_shadow_runtime import WholeCycleShadowController
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    p=argparse.ArgumentParser();p.add_argument('--tag',default='encoder_pose_feedback_v1');p.add_argument('--phase-speed-mode',choices=('fixed1','fixed2','pose_adaptive'));p.add_argument('--margin-mm',type=float);p.add_argument('--case',choices=('nominal','initial_cube_worldX_plus1mm','sliding_mu0p25'));a=p.parse_args()
    if not a.tag.replace('_','').isalnum() or a.margin_mm is not None and not .01<=a.margin_mm<=.5:raise ValueError('Bounded local research options required')
    base=OUT/'next_cycle/handoff_refinement_20261110';root=base/'robust_cycle_preparation_20261111';folder=root/a.tag;bundle=root/'shadow_bundle'
    if folder.exists():raise RuntimeError('Preserve prior whole-cycle controller trials')
    folder.mkdir();(folder/'source_evaluate_whole_cycle_shadow.py').write_bytes(Path(__file__).read_bytes());rows=[]
    for case in ((a.case,) if a.case else ('initial_cube_worldX_plus1mm','sliding_mu0p25','nominal')):
        m,initial,_,_,provenance=assets('active_ccd50');z={k:v.copy() if isinstance(v,np.ndarray) else v for k,v in initial.items()};g=Guard(m);cube=m.geom('cube').id
        if case=='initial_cube_worldX_plus1mm':z['qpos'][-7]+=.001
        if case=='sliding_mu0p25':m.geom_friction[:,0]=.25
        d=restore(m,z);c=WholeCycleShadowController(bundle,observation_mode='simulator_fixture')
        if a.phase_speed_mode:c.meta['phase_speed_mode']=a.phase_speed_mode
        if a.margin_mm is not None:c.meta['MF_static_margin_mm']=a.margin_mm
        sub=folder/case;sub.mkdir();np.savez_compressed(sub/'initial_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time)
        goals=[];phases=[];trajectory=[d.qpos.copy()];fixture=[];stop=[];seq=0;shield_events=slow_events=0;latencies=[];trace=[];cert=SweepCertificate(m,g.hand,.3);count=[0,0];supported=[0,0];clear=[0,0];minimum=[float('inf')]*2;Cs=[None,None];Bs=[None,None];shift=[0.,0.];drift=[0.,0.];first_error=None
        for step in range(13000):
            measured=np.empty(16);measured[SIM_IDS-1]=np.rint(d.qpos[:16]/TICK+2048);quat=np.empty(4);mujoco.mju_mat2Quat(quat,d.geom_xmat[cube])
            sample=dict(sequence=seq,sample_time=seq*.075,pose_time=seq*.075,frame_id='leap_sim_world',palm_point='policy_palm_position',measured_ticks=measured,commanded_ticks=motor(d.ctrl),palm_position=np.array([.11,0.,.03]),cube_position=d.geom_xpos[cube].copy(),cube_quaternion=quat,observation_origin='simulator_fixture')
            began=time.perf_counter()
            try:proposal=c.propose(sample)
            except ValueError as error:stop=['shadow_input_or_goal_contract',str(error)];break
            latencies.append((time.perf_counter()-began)*1000)
            if proposal['state']=='warming':
                fixture.append((sample,None));seq+=1;continue
            command=np.asarray(proposal['goal_ticks']);phase=proposal['reference_index'];previous_phase=c.phase;c.commit_shadow(seq,command);seq+=1;fixture.append((sample,command.copy()))
            shield_events+=proposal.get('details',{}).get('clearance',{}).get('applied',False);slow_events+=proposal.get('details',{}).get('phase_rate_ticks',2)==1
            assert np.max(abs(command-motor(d.ctrl)))<=2+1e-9 and np.all(command>=g.lo) and np.all(command<=g.hi)
            d.ctrl[:]=ticks_to_sim(command);goals.append(command.copy());phases.append(phase);which=0 if 3351<=phase<3405 else (1 if 5035<=phase<5089 else None)
            for _ in range(75):
                mujoco.mj_step(m,d);stop=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if which is not None:
                    C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
                    if Cs[which] is None:Cs[which]=C.copy();Bs[which]=B.copy()
                    count[which]+=1;supported[which]+=sample_model(m,d,g.hand,cube)['supported_sample'];lower,bad=cert.query(d,C,B);minimum[which]=min(minimum[which],lower);clear[which]+=not bad and lower>.3
                    shift[which]=max(shift[which],float(np.linalg.norm(C-Cs[which])*1000));drift[which]=max(drift[which],orientation_error_deg(B,Bs[which]))
                if stop:break
            trajectory.append(d.qpos.copy())
            if previous_phase<3405<=c.phase:first_error=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90.))
            if len(goals)%400==0 or stop or c.phase>=5089:
                entry=dict(case=case,steps=len(goals),reference_index=c.phase,clearance_events=shield_events,slow_events=slow_events,stop=stop,cube_center_world_m=d.geom_xpos[cube].tolist(),worst=g.worst);trace.append(entry);save_json(sub/'progress.json',entry);print(json.dumps(entry),flush=True)
            if stop or c.phase>=5089:break
        if not stop and c.phase<5089:stop=['bounded_trial_incomplete']
        B=d.geom_xmat[cube].reshape(3,3);error=orientation_error_deg(B,upright_yaw_target(initial['initial_cube_rotation'],initial['up'],180.));tilt=face_up_error_deg(B,5,initial['up']);station=np.load(base/'initial_to_reconfigured_first90/end_state.npz',allow_pickle=False)['qpos'][-7:-4];center=float(np.linalg.norm(d.geom_xpos[cube]-station)*1000)
        passed=bool(not stop and first_error is not None and first_error<5 and error<5 and tilt<5 and all(n==4050 and k==4050 and s/4050>.8 and x<2 and b<3 for n,k,s,x,b in zip(count,clear,supported,shift,drift)))
        np.savez_compressed(sub/'research_goals.npz',start=c.start,goals=goals,reference_phases=phases);np.savez_compressed(sub/'trajectory.npz',qpos=trajectory,control_seconds=.075);np.savez_compressed(sub/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time)
        names=('measured_ticks','commanded_ticks','palm_position','cube_position','cube_quaternion');data={name:np.asarray([s[name] for s,_ in fixture]) for name in names};data.update(sequence=[s['sequence'] for s,_ in fixture],sample_time=[s['sample_time'] for s,_ in fixture],pose_time=[s['pose_time'] for s,_ in fixture],expected_goal=np.asarray([np.full(16,np.nan) if goal is None else goal for _,goal in fixture]));np.savez_compressed(sub/'observer_fixture.npz',**data)
        row=dict(case=case,path=sub.relative_to(OUT).as_posix(),steps=len(goals),final_reference_index=c.phase,stop=stop,clearance_events=shield_events,slow_events=slow_events,phase_speed_mode=c.meta['phase_speed_mode'],MF_static_margin_mm=c.meta['MF_static_margin_mm'],first90_error_deg=first_error,final180_error_deg=error,tilt_deg=tilt,first_station_error_mm=center,hold_samples=count,hold_upper_certified_samples=clear,hold_supported_samples=supported,hold_minimum_upper_lower_mm=[v if np.isfinite(v) else None for v in minimum],hold_shift_mm=shift,hold_drift_deg=drift,two_turn_reached_location_holds_passed=passed,same_first_station_cycle_passed=bool(passed and center<2),proposal_latency_ms=dict(median=float(np.median(latencies)),p95=float(np.percentile(latencies,95)),maximum=max(latencies),not_a_realtime_certificate=True),trace=trace,worst=g.worst,terminal_guard=g.last)
        rows.append(row);save_json(sub/'review.json',row);print(json.dumps({k:v for k,v in row.items() if k not in ('trace','terminal_guard')}),flush=True)
    for name in ('whole_cycle_shadow_runtime.py','cycle_joint_clearance.py'):(folder/('source_'+name)).write_bytes((OUT/name).read_bytes())
    save_json(folder/'review.json',dict(status='whole_cycle_encoder_pose_feedback_model_trials',bundle=bundle.relative_to(OUT).as_posix(),bundle_metadata_sha256=sha(bundle/'bundle.json'),model_provenance=provenance,phase_speed_mode_override=a.phase_speed_mode,MF_margin_override_mm=a.margin_mm,rows=rows,existing_SAC_prefix_then_saved_teacher_suffix=True,kinematic_pair_screen_is_not_a_dynamic_certificate=True,integer_encoder_and_exact_model_cube_pose=True,original_guards_and_joint_bounds_preserved=True,actual_state_resets_after_initial_placement=0,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
