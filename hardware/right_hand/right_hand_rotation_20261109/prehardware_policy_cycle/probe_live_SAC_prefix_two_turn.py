"""Reuse the packaged SAC prefix with fresh model observations, then suffix.

The prefix is live feedback, the remaining suffix is saved model-teacher
commands under the unchanged governor. This distinction is preserved in output.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha
from shadow_runtime import SIM_IDS,TICK
from staged_shadow_runtime import AdmissibleShadowController
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'robust_cycle_preparation_20261111/live_SAC_prefix'
    if folder.exists():raise RuntimeError('Preserve live-prefix evidence')
    source=base/'initial_second_turn_hold_replay/research_goals.npz';parent=np.load(source,allow_pickle=False);reference=parent['goals'];prefix=1700;bundle=OUT/'next_cycle/staged_shadow_bundle_v2_ccd50'
    folder.mkdir(parents=True);(folder/'source_probe_live_SAC_prefix_two_turn.py').write_bytes(Path(__file__).read_bytes());rows=[]
    for case in ('nominal','initial_cube_worldX_plus1mm','sliding_mu0p25'):
        m,initial,_,_,provenance=assets('active_ccd50');z={k:v.copy() if isinstance(v,np.ndarray) else v for k,v in initial.items()};g=Guard(m);cube=m.geom('cube').id
        if case=='initial_cube_worldX_plus1mm':z['qpos'][-7]+=.001
        if case=='sliding_mu0p25':m.geom_friction[:,0]=.25
        d=restore(m,z);c=AdmissibleShadowController(bundle);seq=0;stop=[];goals=[];trajectory=[d.qpos.copy()];count=[0,0];supported=[0,0];clear=[0,0];minimum=[float('inf')]*2;Cs=[None,None];Bs=[None,None];shift=[0.,0.];drift=[0.,0.];first_error=None;trace=[];certificate=SweepCertificate(m,g.hand,.3)
        def sensor_sample():
            measured=np.empty(16);measured[SIM_IDS-1]=np.rint(d.qpos[:16]/TICK+2048);q=np.empty(4);mujoco.mju_mat2Quat(q,d.geom_xmat[cube])
            return dict(sequence=seq,sample_time=seq*.075,pose_time=seq*.075,frame_id='leap_sim_world',palm_point='policy_palm_position',measured_ticks=measured,commanded_ticks=motor(d.ctrl),palm_position=np.array([.11,0.,.03]),cube_position=d.geom_xpos[cube].copy(),cube_quaternion=q)
        for _ in range(2):assert c.propose(sensor_sample())['state']=='warming';seq+=1
        for index in range(len(reference)):
            if index<prefix:
                proposal=c.propose(sensor_sample());command=np.asarray(proposal['goal_ticks']);c.commit_shadow(seq,command);seq+=1
            else:
                previous=motor(d.ctrl);command=previous+np.clip(reference[index]-previous,-2,2)
            assert np.max(abs(command-motor(d.ctrl)))<=2+1e-9 and np.all(command>=g.lo) and np.all(command<=g.hi)
            d.ctrl[:]=ticks_to_sim(command);goals.append(command.copy());which=0 if 3351<=index<3405 else (1 if index>=len(reference)-54 else None)
            for _ in range(75):
                mujoco.mj_step(m,d);stop=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if which is not None:
                    C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
                    if Cs[which] is None:Cs[which]=C.copy();Bs[which]=B.copy()
                    count[which]+=1;supported[which]+=sample_model(m,d,g.hand,cube)['supported_sample'];lower,bad=certificate.query(d,C,B);minimum[which]=min(minimum[which],lower);clear[which]+=not bad and lower>.3
                    shift[which]=max(shift[which],float(np.linalg.norm(C-Cs[which])*1000));drift[which]=max(drift[which],orientation_error_deg(B,Bs[which]))
                if stop:break
            trajectory.append(d.qpos.copy())
            if (index+1)%500==0:print(json.dumps(dict(case=case,goals=index+1)),flush=True)
            if index==3404:first_error=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90.))
            if index+1 in (1600,1700,1750,1800,1830,1900,2000,2100,2200) or stop:trace.append(dict(after_goal=index+1,cube_center_world_m=d.geom_xpos[cube].tolist(),guard=g.last,whole_hand_support=sample_model(m,d,g.hand,cube)))
            if stop:break
        differences=np.any(np.asarray(goals)[:min(prefix,len(goals))]!=reference[:min(prefix,len(goals))],axis=1);nominal_exact=case=='nominal'
        if nominal_exact:assert not np.any(differences),'Nominal prefix differs from archived independently generated commands'
        B=d.geom_xmat[cube].reshape(3,3);error=orientation_error_deg(B,upright_yaw_target(initial['initial_cube_rotation'],initial['up'],180.));tilt=face_up_error_deg(B,5,initial['up'])
        passed=bool(not stop and len(goals)==len(reference) and first_error is not None and first_error<5 and error<5 and tilt<5 and all(n==k==4050 and s/4050>.8 and x<2 and a<3 for n,k,s,x,a in zip(count,clear,supported,shift,drift)))
        sub=folder/case;sub.mkdir();np.savez_compressed(sub/'research_goals.npz',start=parent['start'],goals=goals);np.savez_compressed(sub/'trajectory.npz',qpos=trajectory,control_seconds=.075);np.savez_compressed(sub/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time)
        row=dict(case=case,path=sub.relative_to(OUT).as_posix(),steps=len(goals),stop=stop,live_prefix_steps=min(prefix,len(goals)),changed_prefix_goal_rows=int(np.count_nonzero(differences)),nominal_prefix_exact_match=nominal_exact,hold_samples=count,hold_upper_certified_samples=clear,hold_supported_samples=supported,hold_minimum_upper_lower_mm=[v if np.isfinite(v) else None for v in minimum],hold_shift_mm=shift,hold_drift_deg=drift,first90_error_deg=first_error,final180_error_deg=error,tilt_deg=tilt,two_turn_reached_location_holds_passed=passed,trace=trace,worst=g.worst,terminal_guard=g.last)
        rows.append(row);save_json(sub/'review.json',row);print(json.dumps({k:v for k,v in row.items() if k not in ('trace','terminal_guard')}),flush=True)
    save_json(folder/'review.json',dict(status='live_existing_SAC_prefix_then_saved_teacher_suffix',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),bundle=bundle.relative_to(OUT).as_posix(),bundle_metadata_sha256=sha(bundle/'bundle.json'),actor_sha256=sha(bundle/'actor.npz'),model_provenance=provenance,rows=rows,prefix_live_feedback_steps=prefix,suffix_is_saved_teacher_not_live_SAC=True,integer_encoder_and_exact_model_cube_observations=True,model_history_warmup_is_not_real_camera_freshness=True,original_guards_and_bounds_preserved=True,actual_state_resets_after_initial_placement=0,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
