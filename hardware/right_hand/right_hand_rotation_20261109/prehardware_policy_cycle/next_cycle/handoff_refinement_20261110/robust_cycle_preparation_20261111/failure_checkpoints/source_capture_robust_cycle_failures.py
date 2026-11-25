"""Replay failing perturbations and archive exact pre-failure checkpoints.

Only initial counterfactuals and declared sliding friction differ. No guard,
geometry, actuator setting or command limit is changed; devices stay closed.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg

BASE=OUT/'next_cycle/handoff_refinement_20261110'
FOLDER=BASE/'robust_cycle_preparation_20261111'


def state(d,initial):
    return dict(**{k:getattr(d,k).copy() for k in BOUND},time=float(d.time),up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])


def main():
    folder=FOLDER/'failure_checkpoints'
    if folder.exists():raise RuntimeError('Preserve failure checkpoint evidence')
    folder.mkdir(parents=True);(folder/'source_capture_robust_cycle_failures.py').write_bytes(Path(__file__).read_bytes())
    source=BASE/'initial_second_turn_hold_replay/research_goals.npz';proof=BASE/'initial_second_turn_hold_replay/review.json';prior=json.loads(proof.read_text());assert prior['two_rotations_and_clear_holds_at_reached_locations_passed']
    goals=np.load(source,allow_pickle=False)['goals'];poses=np.load(source.parent/'trajectory.npz',allow_pickle=False)['qpos'];rows=[]
    for name in ('nominal','initial_cube_worldX_plus1mm','sliding_mu0p25'):
        m,initial,_,_,provenance=assets('active_ccd50');z={k:v.copy() if isinstance(v,np.ndarray) else v for k,v in initial.items()};original_friction=m.geom_friction.copy()
        if name=='initial_cube_worldX_plus1mm':z['qpos'][-7]+=.001
        if name=='sliding_mu0p25':m.geom_friction[:,0]=.25
        assert np.array_equal(m.geom_friction[:,1:],original_friction[:,1:])
        sub=folder/name;sub.mkdir();d=restore(m,z);g=Guard(m);cube=m.geom('cube').id;np.savez_compressed(sub/'initial_state.npz',**state(d,initial))
        snapshots={k:[] for k in BOUND};indices=[];times=[];guards=[];trace=[];stops=[];previous=motor(d.ctrl);before_bad=None
        def record(index):
            indices.append(index);times.append(float(d.time));guards.append((g.gap,g.longest_gap))
            for k in BOUND:snapshots[k].append(getattr(d,k).copy())
        record(0)
        for index,command in enumerate(goals[:2432]):
            assert np.max(abs(command-previous))<=2+1e-9 and np.all(command>=g.lo) and np.all(command<=g.hi)
            d.ctrl[:]=ticks_to_sim(command);previous=command.copy()
            for micro in range(75):
                before=state(d,initial)
                mujoco.mj_step(m,d);stops=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stops=list(stops)+['cube_translation_over80mm']
                if stops:before_bad=before;break
            number=index+1
            if number>=1500 and number%10==0 or number in (1700,1820,1828,1830,1834,2100,2200,2250,2270,2432) or stops:
                record(number);load=sample_model(m,d,g.hand,cube);q=poses[number];ref=restore(m,dict(initial,qpos=q,qvel=np.zeros(m.nv),ctrl=d.ctrl.copy()))
                trace.append(dict(after_goal=number,error_to_nominal_position_mm=float(np.linalg.norm(d.geom_xpos[cube]-ref.geom_xpos[cube])*1000),error_to_nominal_orientation_deg=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),ref.geom_xmat[cube].reshape(3,3)),whole_hand_support=load,guard=dict(g.last)))
            if number%400==0:print(json.dumps(dict(case=name,goals=number)),flush=True)
            if stops:break
        if name=='nominal':
            assert not stops and np.array_equal(d.qpos,poses[number])
        else:
            earlier=json.loads((BASE/'initial_two_turn_finite_perturbations'/name/'review.json').read_text());end=np.load(BASE/'initial_two_turn_finite_perturbations'/name/'end_state.npz',allow_pickle=False)
            assert earlier['executed_goals']==number and earlier['stop']==stops
            assert all(np.array_equal(getattr(d,k),end[k]) for k in BOUND)
        np.savez_compressed(sub/'state_pool.npz',indices=indices,time=times,guard_gap=np.asarray(guards),**snapshots)
        np.savez_compressed(sub/'end_state.npz',**state(d,initial))
        if before_bad is not None:np.savez_compressed(sub/'before_rejected_1ms_state.npz',**before_bad)
        case=dict(case=name,path=sub.relative_to(OUT).as_posix(),model_provenance=provenance,sliding_friction_only_modified=name=='sliding_mu0p25',sliding_friction_hypothesis=.25 if name=='sliding_mu0p25' else None,initial_cube_world_offset_mm=[1.,0.,0.] if name=='initial_cube_worldX_plus1mm' else [0.,0.,0.],goals=number,stop=stops,exact_prior_failure_reproduced=name!='nominal',state_pool_sha256=sha(sub/'state_pool.npz'),trace=trace,worst=g.worst,terminal_guard=g.last)
        save_json(sub/'review.json',case);rows.append(case);print(json.dumps({k:v for k,v in case.items() if k not in ('trace','model_provenance')}),flush=True)
    model,initial,_,_,_=assets('active_ccd50');g=Guard(model);np.savez_compressed(FOLDER/'command_contract.npz',lower=g.lo,upper=g.hi,qlo=g.qlo,qhi=g.qhi,sim_motor_ids=np.array((8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1)),start=np.load(source,allow_pickle=False)['start'])
    save_json(folder/'review.json',dict(status='exact_perturbed_failure_checkpoints',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),nominal_proof_sha256=sha(proof),rows=rows,whole_hand_support_includes_palm=True,actual_state_resets_after_initial_placement=0,original_guards_and_bounds_preserved=True,friction_not_measured=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
