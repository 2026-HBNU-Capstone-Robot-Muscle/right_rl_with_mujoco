"""Replay initial placement through the actually reached90-degree hold.

The reached workstation is5.6mm from the old canonical target. This certifies
the named alternative workstation only; old canonical failures are retained.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'initial_to_reached_workstation90'
    if folder.exists():raise RuntimeError('Preserve earlier full replay')
    model,initial,_,_,provenance=assets('active_ccd50');data=restore(model,initial);guard=Guard(model);cube=model.geom('cube').id
    parent=OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_active_ccd50/research_trace.npz'
    poolpath=base/'early_CCD50_states/state_pool.npz';pool=np.load(poolpath,allow_pickle=False);i=int(np.flatnonzero(pool['indices']==1700)[0])
    poolstate={k:pool[k][i] for k in BOUND};checkpoint=base/'first90_finish_checkpoint';wp=base/'first90_RF_shape_before_adduction/RF_park_MF_then_RF_shape_then_adduct'
    cpgoals=np.load(checkpoint/'goals.npz',allow_pickle=False)['goals'];assert len(cpgoals)==732
    first=np.load(wp/'research_goals.npz',allow_pickle=False)['goals'][:529]
    state1=dict(np.load(wp/'phase1_end_state.npz',allow_pickle=False));station=state1['qpos'][-7:-4].copy();holdgoals=np.tile(motor(state1['ctrl']),(54,1))
    parts=[('policy_prefix1700',np.load(parent,allow_pickle=False)['goals'][:1700],poolstate),
        ('transport_feedback732',cpgoals,dict(np.load(checkpoint/'state.npz',allow_pickle=False))),
        ('RF_park_and_MF_open529',first,state1),
        ('reached_station_hold54',holdgoals,dict(np.load(base/'actual_reached_workstation_hold_audit/first90_MF_open_RF_park/end_state.npz',allow_pickle=False)))]
    folder.mkdir();(folder/'source_replay_first_reached_workstation.py').write_bytes(Path(__file__).read_bytes());all_goals=[];seams=[];trajectory=[data.qpos.copy()];previous=motor(data.ctrl)
    certificate=SweepCertificate(model,guard.hand,.3);count=certified=supported=0;minimum=float('inf');drift=shift=0.;holdB=None
    for label,goals,expected in parts:
        for goal in goals:
            assert np.max(abs(goal-previous))<=2 and np.all(goal>=guard.lo) and np.all(goal<=guard.hi)
            data.ctrl[:]=ticks_to_sim(goal);previous=goal.copy();all_goals.append(goal.copy())
            for _ in range(75):
                mujoco.mj_step(model,data);stop=guard.check(data)
                assert not stop,(label,len(all_goals),guard.last)
                assert np.linalg.norm(data.qpos[-7:-4]-initial['initial_cube_center'])<=.08
                if label=='reached_station_hold54':
                    B=data.geom_xmat[cube].reshape(3,3);C=data.geom_xpos[cube]
                    if holdB is None:holdB=B.copy()
                    count+=1;supported+=sample_model(model,data,guard.hand,cube)['supported_sample'];value,bad=certificate.query(data,C,B);minimum=min(minimum,value);certified+=not bad and value>.3
                    drift=max(drift,orientation_error_deg(B,holdB));shift=max(shift,float(np.linalg.norm(C-station)*1000))
            trajectory.append(data.qpos.copy())
        differences={k:float(np.max(abs(getattr(data,k)-expected[k]))) if expected[k].size else 0. for k in BOUND}
        assert max(differences.values())==0.,(label,differences)
        seams.append(dict(segment=label,goals=len(goals),after_goal=len(all_goals),bound_state_errors=differences))
        print(json.dumps(dict(segment=label,goals=len(all_goals),exact_bound_state_match=True)),flush=True)
    target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90.);error=orientation_error_deg(data.geom_xmat[cube].reshape(3,3),target);tilt=face_up_error_deg(data.geom_xmat[cube].reshape(3,3),5,initial['up'])
    canonical=np.load(OUT/'baseline_archived_zero/end_state.npz',allow_pickle=False)['qpos'][-7:-4];canonical_error=float(np.linalg.norm(data.geom_xpos[cube]-canonical)*1000)
    passed=bool(count==certified==4050 and supported/4050>.8 and minimum>.3 and error<5 and tilt<5 and shift<2 and drift<3)
    np.savez_compressed(folder/'research_goals.npz',start=motor(initial['ctrl']),goals=all_goals)
    np.savez_compressed(folder/'trajectory.npz',qpos=trajectory,control_seconds=.075)
    np.savez_compressed(folder/'end_state.npz',**{k:getattr(data,k) for k in BOUND},time=data.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
    sourcefiles=[parent,poolpath,checkpoint/'goals.npz',checkpoint/'state.npz',wp/'research_goals.npz',wp/'phase1_end_state.npz']
    save_json(folder/'review.json',dict(status='initial_placement_to_explicit_reached_workstation90',source_files_sha256={p.relative_to(OUT).as_posix():sha(p) for p in sourcefiles},model_provenance=provenance,
        steps=len(all_goals),seams=seams,actual_state_resets_after_initial_placement=0,first_rotation_and_reached_station_hold_passed=passed,
        final_orientation_error_deg=error,tilt_deg=tilt,canonical_station_error_mm=canonical_error,alternative_reached_station_world_m=station.tolist(),
        hold_samples=count,certified_upper_samples=certified,minimum_upper_lower_mm=minimum,whole_hand_support_fraction=supported/max(1,count),hold_shift_mm=shift,hold_drift_deg=drift,
        original_canonical_station_2mm_task_passed=bool(passed and canonical_error<2),original_canonical_failures_preserved=True,left_hand_workstation_reach_and_actual_load_not_validated=True,
        policy_prefix_and_model_teacher_are_separate=True,prefix_saved_policy_goals=1700,model_teacher_goals=1261,position_hold_goals=54,
        second_rotation_passed=False,full_two_turn_cycle_passed=False,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False,worst=guard.worst))
    print(json.dumps(dict(steps=len(all_goals),passed_at_reached_station=passed,canonical_error_mm=canonical_error,error_deg=error,minimum_upper_mm=minimum)),flush=True)


if __name__=='__main__':main()
