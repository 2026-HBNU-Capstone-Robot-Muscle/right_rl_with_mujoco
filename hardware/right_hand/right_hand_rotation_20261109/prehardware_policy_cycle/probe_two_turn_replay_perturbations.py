"""Finite perturbation checks of the saved path, not closed-loop SAC tests.

Each counterfactual is initialized once before the first command; thereafter the
object is free and all original guards apply. No measured friction is assumed.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,ticks_to_sim,save_json,sha,TICK
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';parent=base/'initial_second_turn_hold_replay';folder=base/'initial_two_turn_finite_perturbations'
    prior=json.loads((parent/'review.json').read_text());z=np.load(parent/'research_goals.npz',allow_pickle=False);goals=z['goals'];start=z['start']
    if folder.exists():raise RuntimeError('Preserve earlier finite robustness evidence')
    folder.mkdir();(folder/'source_probe_two_turn_replay_perturbations.py').write_bytes(Path(__file__).read_bytes());rows=[]
    for name in ('sliding_mu0p25','initial_cube_worldX_plus1mm','initial_MF_TH_one_tick'):
        model,initial,_,_,provenance=assets('active_ccd50');initial={k:v.copy() if isinstance(v,np.ndarray) else v for k,v in initial.items()};original_friction=model.geom_friction.copy()
        cube=model.geom('cube').id
        if name=='sliding_mu0p25':model.geom_friction[:,0]=.25
        if name=='initial_cube_worldX_plus1mm':initial['qpos'][-7]+=.001
        if name=='initial_MF_TH_one_tick':initial['qpos'][4:8]+=TICK;initial['qpos'][12:16]-=TICK
        assert np.array_equal(model.geom_friction[:,1:],original_friction[:,1:])
        # The original world station is fixed by nominal first hold, never by
        # this counterfactual's observed final object position.
        first=np.load(base/'initial_to_reconfigured_first90/end_state.npz',allow_pickle=False);station=first['qpos'][-7:-4]
        d=restore(model,initial);g=Guard(model);cert=SweepCertificate(model,g.hand,.3);target90=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90.);target180=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],180.)
        count=[0,0];supported=[0,0];clear=[0,0];minimum=[float('inf')]*2;Cs=[None,None];Bs=[None,None];shift=[0.,0.];drift=[0.,0.];stop=[];done=0;previous=start.copy()
        for step,command in enumerate(goals):
            assert np.max(abs(command-previous))<=2+1e-9 and np.all(command>=g.lo) and np.all(command<=g.hi)
            d.ctrl[:]=ticks_to_sim(command);previous=command.copy();which=0 if 3351<=step<3405 else (1 if step>=len(goals)-54 else None)
            for _ in range(75):
                mujoco.mj_step(model,d);stop=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if which is not None:
                    C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
                    if Cs[which] is None:Cs[which]=C.copy();Bs[which]=B.copy()
                    count[which]+=1;supported[which]+=sample_model(model,d,g.hand,cube)['supported_sample'];lower,bad=cert.query(d,C,B);minimum[which]=min(minimum[which],lower);clear[which]+=not bad and lower>.3
                    shift[which]=max(shift[which],float(np.linalg.norm(C-Cs[which])*1000));drift[which]=max(drift[which],orientation_error_deg(B,Bs[which]))
                if stop:break
            done=step+1
            if step==3404:first_error=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),target90)
            if stop:break
        B=d.geom_xmat[cube].reshape(3,3);error=orientation_error_deg(B,target180);tilt=face_up_error_deg(B,5,initial['up']);center=float(np.linalg.norm(d.geom_xpos[cube]-station)*1000)
        passed=bool(not stop and done==len(goals) and all(c==4050 and k==4050 and s/4050>.8 and x<2 and a<3 for c,k,s,x,a in zip(count,clear,supported,shift,drift)) and first_error<5 and error<5 and tilt<5)
        sub=folder/name;sub.mkdir();np.savez_compressed(sub/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time)
        r=dict(path=sub.relative_to(OUT).as_posix(),case=name,executed_goals=done,total_goals=len(goals),stop=stop,hold_samples=count,hold_supported_samples=supported,hold_upper_certified_samples=clear,hold_minimum_upper_lower_mm=[v if np.isfinite(v) else None for v in minimum],hold_shift_mm=shift,hold_drift_deg=drift,final180_error_deg=error,final_tilt_deg=tilt,first_station_error_mm=center,two_rotation_clear_holds_at_reached_locations_passed=passed,same_first_station_cycle_passed=bool(passed and center<2),worst=g.worst,terminal_guard=g.last)
        rows.append(r);save_json(sub/'review.json',r);print(json.dumps({k:v for k,v in r.items() if k not in ('worst','terminal_guard')}),flush=True)
    save_json(folder/'review.json',dict(status='finite_saved_two_turn_path_perturbations',source=parent.relative_to(OUT).as_posix()+'/research_goals.npz',source_sha256=sha(parent/'research_goals.npz'),nominal_report_sha256=sha(parent/'review.json'),rows=rows,actual_state_resets_after_initial_placement=0,nominal_path_not_reoptimized=True,saved_path_not_a_new_closed_loop_SAC_policy=True,friction_is_hypothesis_not_measurement=True,first_station_2mm_target_not_changed=True,finite_cases_not_general_robustness_certificate=True,original_guards_and_joint_bounds_preserved=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
