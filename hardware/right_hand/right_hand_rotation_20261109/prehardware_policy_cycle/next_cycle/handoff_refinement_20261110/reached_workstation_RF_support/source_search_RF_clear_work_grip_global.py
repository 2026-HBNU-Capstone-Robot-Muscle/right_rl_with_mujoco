"""Global RF contact/clearance search at an IF/base-supported model grip.

No fixed material contact patch is assumed. The cube is fixed only in static
proposals; every selected arrival and hold is free native dynamics. This is a
conditional suffix study, not an initial-placement cycle or hardware test.
"""
from pathlib import Path
import argparse
import json
import time
from collections import deque
import numpy as np
import mujoco
from scipy.optimize import differential_evolution
from native_cycle import OUT, ROOT, assets, restore, Guard, BOUND, motor, ticks_to_sim, TICK, SIM_TO_MOTOR_IDS, save_json, sha
from whole_hand_support import sample_model
from convex_pair_gap import ConvexPairGap
from cube_task_frame_20261105 import upright_yaw_target, orientation_error_deg, face_up_error_deg
from upper_layer_cylinder_clearance_20261105 import precompute, gaps
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--source',required=True);parser.add_argument('--tag',default='RF_clear_global_after_index')
    parser.add_argument('--bounded-distance',action='store_true');parser.add_argument('--station-scope',choices=('canonical','source'),default='canonical')
    args=parser.parse_args();source=(OUT/args.source).resolve()
    if not source.is_relative_to(OUT.resolve()) or not args.tag.replace('_','').isalnum():raise ValueError('Invalid research input')
    folder=OUT/'next_cycle/handoff_refinement_20261110'/args.tag
    if folder.exists():raise RuntimeError('Preserve previous RF global study')
    model,initial,_,_,provenance=assets('active_ccd50');guard=Guard(model);cube=model.geom('cube').id
    z=dict(np.load(source,allow_pickle=False));data=restore(model,z);center=data.geom_xpos[cube].copy();rotation=data.geom_xmat[cube].reshape(3,3).copy()
    rf=np.arange(8,12);sid=np.array(SIM_TO_MOTOR_IDS)-1;lag=data.ctrl-data.qpos[:16]
    low=np.maximum(guard.qlo,(np.ceil(guard.lo[sid])+2-2048)*TICK-lag)
    high=np.minimum(guard.qhi,(np.floor(guard.hi[sid])-2-2048)*TICK-lag)
    geometry=precompute(model,guard.hand)
    rf_geometry=[item for item in geometry if (model.body(int(model.geom_bodyid[item[0]])).name or '').startswith('rf_')]
    eligible=[model.geom(name).id for name in ('rf_tip','rf_ds_collision_1','rf_md_collision_1')]
    if args.bounded_distance:
        from planning_pair_gap import PlanningPairGap
        distance=PlanningPairGap(model,[cube]+eligible)
    else:distance=ConvexPairGap(model,[cube]+eligible)
    recorded=ROOT/'ring_reference_readonly_20261109_124719_945552/result.json'
    measured={row['id']:row['position_ticks'] for row in json.loads(recorded.read_text())['snapshots'][0]['motors']}
    taught=np.clip((np.array([measured[mid] for mid in np.array(SIM_TO_MOTOR_IDS)[rf]])-2048)*TICK,low[rf],high[rf])
    folder.mkdir();(folder/'source_search_RF_clear_work_grip_global.py').write_bytes(Path(__file__).read_bytes())
    rows=[];candidates=[];began=time.perf_counter()
    for geom in eligible:
        proposal=restore(model,z)
        def measure(x):
            proposal.qpos[:]=z['qpos'];proposal.qpos[rf]=x;proposal.ctrl[:]=proposal.qpos[:16]+lag
            mujoco.mj_forward(model,proposal)
            support,own=guard.contacts(proposal)
            separation=distance.gap(proposal,cube,geom)*1000
            clear=min(value for _,value in gaps(model,proposal,center,rotation,rf_geometry,True))
            penetration=max((-float(c.dist)*1000 for c in proposal.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            return separation,clear,support,own,penetration
        def objective(x):
            separation,clear,support,own,penetration=measure(x)
            return 4*abs(separation+.025)+10*max(0.,.8-clear)+2000*max(own.values(),default=0.)+1000*max(support.values(),default=0.)+500*max(0.,penetration-.8)
        generator=np.random.default_rng(702+geom);population=generator.uniform(low[rf],high[rf],size=(48,4))
        population[0]=data.qpos[rf];population[1]=taught
        fit=differential_evolution(objective,list(zip(low[rf],high[rf])),seed=702+geom,init=population,maxiter=100,tol=1e-6,polish=True,workers=1)
        separation,clear,support,own,penetration=measure(fit.x);goal=motor(proposal.ctrl)
        contacts=[]
        for contact in proposal.contact:
            first,second=int(contact.geom1),int(contact.geom2)
            if cube not in (first,second):continue
            other=second if first==cube else first
            if (model.body(int(model.geom_bodyid[other])).name or '').startswith('rf_') and contact.dist<.0001:
                contacts.append(dict(geom=model.geom(other).name,distance_mm=float(contact.dist*1000),cube_local_mm=((contact.pos-center)@rotation*1000).tolist()))
        possible=bool(contacts and max(own.values(),default=0.)<=1e-6 and max(support.values(),default=0.)<=.15 and penetration<=1 and np.all(goal>=guard.lo) and np.all(goal<=guard.hi))
        cert=SweepCertificate(model,guard.hand,.3);lower,bad=cert.query(proposal,center,rotation)
        valid=bool(possible and not bad and lower>.3)
        label=model.geom(geom).name;proposal_path=folder/(label+'_proposal.npz')
        np.savez_compressed(proposal_path,qpos=proposal.qpos,ctrl=proposal.ctrl)
        row=dict(geom=label,nfev=int(fit.nfev),objective=float(fit.fun),signed_axis_gap_mm=separation,RF_screen_mm=clear,
            RF_contacts=contacts,self_pairs_mm=own,cube_penetration_mm=penetration,upper_certificate_lower_bound_mm=lower,
            bad_upper_geoms=[model.geom(int(cert.ids[i])).name for i in bad],static_clear_work_grip_candidate=valid,
            proposal=proposal_path.relative_to(OUT).as_posix(),proposal_sha256=sha(proposal_path),goal_ticks=goal.tolist())
        rows.append(row)
        if valid and all(np.max(abs(goal-item[0]))>10 for item in candidates):candidates.append((goal.copy(),proposal.qpos.copy(),proposal.ctrl.copy(),label))
        save_json(folder/'progress.json',dict(completed_proposals=rows,elapsed_seconds=time.perf_counter()-began))
        print(json.dumps(dict(geom=label,gap_mm=separation,RF_clear_mm=clear,candidate=valid,bad_upper=row['bad_upper_geoms'])),flush=True)
    target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90)
    station=np.load(OUT/'baseline_archived_zero/end_state.npz',allow_pickle=False)['qpos'][-7:-4];canonical_station=station.copy();trials=[]
    if args.station_scope=='source':station=center.copy()
    for base_goal,pose,control,label in candidates:
        prep=restore(model,z);prep.qpos[:]=pose;prep.ctrl[:]=control;mujoco.mj_forward(model,prep)
        contact=next(c for c in prep.contact if cube in (int(c.geom1),int(c.geom2)) and
            (model.body(int(model.geom_bodyid[int(c.geom2) if int(c.geom1)==cube else int(c.geom1)])).name or '').startswith('rf_'))
        other=int(contact.geom2) if int(contact.geom1)==cube else int(contact.geom1)
        normal=contact.frame[:3]*(1 if int(contact.geom1)==cube else -1)
        jac=np.zeros((3,model.nv));mujoco.mj_jac(model,prep,jac,None,contact.pos,int(model.geom_bodyid[other]))
        closing=-jac[:,:16].T@normal/model.actuator_gainprm[:,0]
        for preload in (0.,.4,.8):
            desired=control.copy();desired[rf]+=closing[rf]*preload;goal=motor(desired)
            if np.any(goal<guard.lo) or np.any(goal>guard.hi):continue
            data=restore(model,z);checker=Guard(model);start=motor(data.ctrl);previous=start.copy()
            count=max(1,int(np.ceil(np.max(abs(goal-start))/2)));goals=[];stop=[];history=deque(maxlen=4050)
            samples=supported=rf_contact_count=rf_loaded=0;angle_drift=position_drift=0.;hold_B=hold_C=None
            for step in range(count+134):
                command=np.rint(start+(goal-start)*min(step+1,count)/count).astype(int)
                if np.max(abs(command-previous))>2 or np.any(command<checker.lo) or np.any(command>checker.hi):raise AssertionError('Command contract changed')
                data.ctrl[:]=ticks_to_sim(command);previous=command;goals.append(command.copy())
                for _ in range(75):
                    mujoco.mj_step(model,data);stop=checker.check(data)
                    if np.linalg.norm(data.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                    if step>=count+80:
                        sample=sample_model(model,data,checker.hand,cube);samples+=1;supported+=sample['supported_sample']
                        force=sum(value['normal_N'] for name,value in sample['by_geom'].items() if (model.body(int(model.geom_bodyid[model.geom(name).id])).name or '').startswith('rf_'))
                        rf_contact_count+=force>1e-5;rf_loaded+=force>.2
                        C=data.geom_xpos[cube];B=data.geom_xmat[cube].reshape(3,3)
                        if hold_B is None:hold_B=B.copy();hold_C=C.copy()
                        angle_drift=max(angle_drift,orientation_error_deg(B,hold_B));position_drift=max(position_drift,float(np.linalg.norm(C-hold_C)*1000));history.append(data.qpos.copy())
                    if stop:break
                if stop:break
            error=orientation_error_deg(data.geom_xmat[cube].reshape(3,3),target);tilt=face_up_error_deg(data.geom_xmat[cube].reshape(3,3),5,initial['up'])
            station_error=float(np.linalg.norm(data.geom_xpos[cube]-station)*1000)
            pose_pass=bool(not stop and samples==4050 and supported/4050>.8 and angle_drift<3 and position_drift<2 and error<5 and tilt<5 and station_error<2)
            certified=0;lower=None;bad=[];cert=SweepCertificate(model,checker.hand,.3)
            if pose_pass:
                check_data=restore(model,z);check_data.qpos[:]=history[-1];mujoco.mj_forward(model,check_data)
                lower,bad=cert.query(check_data,check_data.geom_xpos[cube],check_data.geom_xmat[cube].reshape(3,3))
                if not bad and lower>.3:
                    for qpos in history:
                        check_data.qpos[:]=qpos;mujoco.mj_forward(model,check_data)
                        value,bad=cert.query(check_data,check_data.geom_xpos[cube],check_data.geom_xmat[cube].reshape(3,3));lower=min(lower,value);certified+=1
                        if bad or value<=.3:break
            passed=bool(pose_pass and certified==4050 and not bad and lower>.3)
            trial=folder/f'{label}_preload{preload:g}';trial.mkdir()
            np.savez_compressed(trial/'research_goals.npz',start=start,goals=goals)
            np.savez_compressed(trial/'end_state.npz',**{key:getattr(data,key) for key in BOUND},time=data.time,
                up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
            row=dict(path=trial.relative_to(OUT).as_posix(),geom=label,preload_force_hypothesis_N=preload,steps=len(goals),stop=stop,
                hold_samples=samples,whole_hand_support_fraction=supported/max(1,samples),RF_contact_fraction=rf_contact_count/max(1,samples),RF_over_point2N_fraction=rf_loaded/max(1,samples),
                hold_angle_drift_deg=angle_drift,hold_center_drift_mm=position_drift,final_global90_error_deg=error,final_tilt_deg=tilt,
                common_station_error_mm=station_error,pose_and_whole_hand_load_passed=pose_pass,certified_window_samples=certified,
                station_scope=args.station_scope,original_canonical_station_error_mm=float(np.linalg.norm(data.geom_xpos[cube]-canonical_station)*1000),
                upper_certificate_lower_bound_mm=lower,initialized_clear_work_grip_suffix_passed=passed,
                initialized_three_point_contact_and_clearance_passed=bool(passed and rf_contact_count/4050>.8),
                worst=checker.worst,terminal_guard=checker.last,terminal_support=sample_model(model,data,checker.hand,cube))
            save_json(trial/'review.json',row);trials.append(row);print(json.dumps(dict(trial=trial.name,stop=stop,passed=passed,error_deg=error)),flush=True)
    report=dict(status='global_RF_clear_work_grip_geometry_and_free_suffix',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),
        recorded_RF_source_sha256=sha(recorded),recorded_RF_seed_clipped_to_original_bounds_not_new_goal=True,model_provenance=provenance,
        proposals=rows,dynamic_trials=trials,initialized_clear_work_grip_suffix_passed=any(row['initialized_clear_work_grip_suffix_passed'] for row in trials),
        full_initial_placement_two_turn_cycle_passed=False,conditional_endpoint_suffix_only=True,optimizer_budget_is_not_general_infeasibility_proof=True,
        RF_force_point2N_is_diagnostic_not_whole_hand_support_gate=True,other_hand_applied_load_not_evaluated=True,
        station_scope=args.station_scope,old_canonical_failures_not_reclassified=True,bounded_axis_distance_is_planning_cost_only=args.bounded_distance,
        original_guards_geometry_and_joint_limits_preserved=True,devices_opened=0,new_SAC_training_steps=0,hardware_execution_allowed=False,
        elapsed_seconds=time.perf_counter()-began)
    save_json(folder/'review.json',report)


if __name__=='__main__':main()
