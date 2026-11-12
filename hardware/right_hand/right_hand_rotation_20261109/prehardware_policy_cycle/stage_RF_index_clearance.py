"""Clear the unloaded IF tip first, retaining the RF/base-supported model grip.

Only proposal copies fix the cube. Arrival and holding use continuous free
dynamics with the original guards and two-tick/75 ms control limits.
"""
from pathlib import Path
import argparse
import json
import time
import numpy as np
import mujoco
from scipy.optimize import differential_evolution
from native_cycle import OUT, assets, restore, Guard, BOUND, motor, ticks_to_sim, TICK, SIM_TO_MOTOR_IDS, save_json, sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target, orientation_error_deg, face_up_error_deg
from upper_layer_cylinder_clearance_20261105 import precompute, gaps
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--margin',type=float,default=.65)
    parser.add_argument('--tag',default='RF_index_clearance_stage');args=parser.parse_args()
    if not args.tag.replace('_','').isalnum() or not .3<args.margin<10:raise ValueError('Invalid research tag or margin')
    folder=OUT/'next_cycle/handoff_refinement_20261110'/args.tag
    if folder.exists():raise RuntimeError('Preserve previous index-clearance study')
    model,initial,_,_,provenance=assets('active_ccd50');guard=Guard(model);cube=model.geom('cube').id
    source=OUT/'next_cycle/handoff_refinement_20261110/RF_endpoint_CCD50_load_screen/qualified0_end_state.npz'
    z=dict(np.load(source,allow_pickle=False));data=restore(model,z)
    center=data.geom_xpos[cube].copy();rotation=data.geom_xmat[cube].reshape(3,3).copy()
    free=np.arange(1,4);sid=np.array(SIM_TO_MOTOR_IDS)-1;lag=data.ctrl-data.qpos[:16]
    low=np.maximum(guard.qlo,(np.ceil(guard.lo[sid])+2-2048)*TICK-lag)
    high=np.minimum(guard.qhi,(np.floor(guard.hi[sid])-2-2048)*TICK-lag)
    hand_geometry=precompute(model,guard.hand)
    geometry=[item for item in hand_geometry if (model.body(int(model.geom_bodyid[item[0]])).name or '').startswith('if_')]
    index_bodies={int(model.geom_bodyid[item[0]]) for item in geometry}
    folder.mkdir();(folder/'source_stage_RF_index_clearance.py').write_bytes(Path(__file__).read_bytes())
    began=time.perf_counter();rows=[];chosen=[]
    for seed in (13,29):
        proposal=restore(model,z)
        def measure(x):
            proposal.qpos[:]=z['qpos'];proposal.qpos[free]=x;proposal.ctrl[:]=proposal.qpos[:16]+lag
            mujoco.mj_forward(model,proposal)
            support,own=guard.contacts(proposal)
            clear=min(value for _,value in gaps(model,proposal,center,rotation,geometry,True))
            penetration=max((-float(c.dist)*1000 for c in proposal.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            return clear,support,own,penetration
        def objective(x):
            clear,support,own,penetration=measure(x)
            return 10*max(0.,args.margin-clear)+max(own.values(),default=0.)*2000+max(support.values(),default=0.)*1000+max(0.,penetration-.8)*500+.02*float(np.linalg.norm(x-z['qpos'][free]))
        fit=differential_evolution(objective,list(zip(low[free],high[free])),seed=seed,popsize=10,maxiter=80,tol=1e-7,polish=True,workers=1,x0=z['qpos'][free])
        clear,support,own,penetration=measure(fit.x);goal=motor(proposal.ctrl)
        cert=SweepCertificate(model,index_bodies,.3);lower,bad=cert.query(proposal,center,rotation)
        valid=bool(not own and max(support.values(),default=0.)<=.15 and penetration<=1 and not bad and lower>.3)
        row=dict(seed=seed,nfev=int(fit.nfev),objective=float(fit.fun),IF_screen_mm=clear,IF_certificate_lower_mm=lower,
            self_pairs_mm=own,cube_penetration_mm=penetration,static_IF_clearance_candidate=valid,goal_ticks=goal.tolist())
        rows.append(row)
        if valid and all(np.max(abs(goal-old))>10 for old in chosen):chosen.append(goal.copy())
        print(json.dumps(dict(seed=seed,candidate=valid,IF_lower_mm=lower)),flush=True)
    target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90);trials=[]
    for index,goal in enumerate(chosen):
        data=restore(model,z);checker=Guard(model);start=motor(data.ctrl);previous=start.copy()
        count=max(1,int(np.ceil(np.max(abs(goal-start))/2)));goals=[];stop=[];samples=supported=rf_loaded=if_clear=0
        certificate=SweepCertificate(model,index_bodies,.3);minimum=float('inf');angle_drift=position_drift=0.;hold_B=hold_C=None
        for step in range(count+54):
            command=np.rint(start+(goal-start)*min(step+1,count)/count).astype(int)
            if np.max(abs(command-previous))>2 or np.any(command<checker.lo) or np.any(command>checker.hi):raise AssertionError('Command contract changed')
            data.ctrl[:]=ticks_to_sim(command);previous=command;goals.append(command.copy())
            for _ in range(75):
                mujoco.mj_step(model,data);stop=checker.check(data)
                if np.linalg.norm(data.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if step>=count:
                    sample=sample_model(model,data,checker.hand,cube);samples+=1;supported+=sample['supported_sample']
                    rf_loaded+=sum(value['normal_N'] for name,value in sample['by_geom'].items()
                        if (model.body(int(model.geom_bodyid[model.geom(name).id])).name or '').startswith('rf_'))>.2
                    C=data.geom_xpos[cube];B=data.geom_xmat[cube].reshape(3,3)
                    lower,bad=certificate.query(data,C,B);minimum=min(minimum,lower);if_clear+=not bad and lower>.3
                    if hold_B is None:hold_B=B.copy();hold_C=C.copy()
                    angle_drift=max(angle_drift,orientation_error_deg(B,hold_B));position_drift=max(position_drift,float(np.linalg.norm(C-hold_C)*1000))
                if stop:break
            if stop:break
        error=orientation_error_deg(data.geom_xmat[cube].reshape(3,3),target)
        passed=bool(not stop and samples==4050 and supported/4050>.8 and rf_loaded/4050>.8 and if_clear==4050 and angle_drift<3 and position_drift<2 and error<5)
        trial=folder/f'trial{index}';trial.mkdir()
        np.savez_compressed(trial/'research_goals.npz',start=start,goals=goals)
        np.savez_compressed(trial/'end_state.npz',**{key:getattr(data,key) for key in BOUND},time=data.time,
            up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        row=dict(path=trial.relative_to(OUT).as_posix(),steps=len(goals),stop=stop,hold_samples=samples,
            whole_hand_support_fraction=supported/max(1,samples),RF_loaded_fraction=rf_loaded/max(1,samples),IF_clear_samples=if_clear,
            IF_certificate_minimum_lower_mm=minimum if samples else None,hold_angle_drift_deg=angle_drift,hold_center_drift_mm=position_drift,
            final_global90_error_deg=error,final_tilt_deg=face_up_error_deg(data.geom_xmat[cube].reshape(3,3),5,initial['up']),
            initialized_IF_clear_RF_hold_stage_passed=passed,worst=checker.worst,terminal_guard=checker.last,
            initialized_IF_clear_whole_hand_hold_passed=bool(not stop and samples==4050 and supported/4050>.8 and if_clear==4050 and angle_drift<3 and position_drift<2 and error<5),
            terminal_whole_hand_support=sample_model(model,data,checker.hand,cube),
            upper_screen_all_hand_mm=min(value for _,value in gaps(model,data,data.geom_xpos[cube],data.geom_xmat[cube].reshape(3,3),hand_geometry,True)))
        save_json(trial/'review.json',row);trials.append(row);print(json.dumps(dict(trial=index,steps=len(goals),stop=stop,passed=passed)),flush=True)
    report=dict(status='RF_supported_index_clearance_stage',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),model_provenance=provenance,
        proposal_results=rows,dynamic_trials=trials,initialized_stage_passed=any(row['initialized_IF_clear_RF_hold_stage_passed'] for row in trials),
        IF_base_and_other_finger_commands_held=True,optimization_margin_mm=args.margin,RF_point2N_gate_is_separate_from_whole_hand_support=True,
        all_finger_clearance_or_full_cycle_not_certified=True,original_guards_and_limits_preserved=True,
        devices_opened=0,new_SAC_training_steps=0,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began)
    save_json(folder/'review.json',report)


if __name__=='__main__':main()
