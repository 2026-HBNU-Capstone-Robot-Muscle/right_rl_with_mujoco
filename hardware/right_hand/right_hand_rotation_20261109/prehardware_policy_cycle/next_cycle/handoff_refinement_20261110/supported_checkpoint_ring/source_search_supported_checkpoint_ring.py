"""Find RF contacts while the actual CCD50 checkpoint is supported by the hand.

Only IK copies hold a cube pose fixed. Every arrival/hold trial starts at the
actual checkpoint and runs a free cube with all original native guards.
"""
from pathlib import Path
import time
import json
import numpy as np
import mujoco
from scipy.optimize import least_squares
from native_cycle import OUT,ROOT,assets,restore,Guard,motor,ticks_to_sim,BOUND,TICK,SIM_TO_MOTOR_IDS,save_json,sha
from cube_task_frame_20261105 import orientation_error_deg
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from whole_hand_support import sample_model


def main():
    folder=OUT/'next_cycle/handoff_refinement_20261110/supported_checkpoint_ring'
    if folder.exists():raise RuntimeError('Preserve previous RF study')
    m,initial,_,_,provenance=assets('active_ccd50');cube=m.geom('cube').id;checker=Guard(m)
    pool_path=OUT/'next_cycle/handoff_refinement_20261110/early_CCD50_states/state_pool.npz'
    pool=np.load(pool_path,allow_pickle=False);index=int(np.flatnonzero(pool['indices']==1700)[0])
    z=dict(initial);z.update({key:pool[key][index].copy() for key in BOUND});z['time']=float(pool['time'][index])
    d=restore(m,z);q0=d.qpos[:16].copy();lag=d.ctrl-q0;rf=np.arange(8,12);C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy()
    sid=np.array(SIM_TO_MOTOR_IDS)-1
    low=np.maximum(checker.qlo,(np.ceil(checker.lo[sid])+2-2048)*TICK-lag)
    high=np.minimum(checker.qhi,(np.floor(checker.hi[sid])-2-2048)*TICK-lag)
    cache=precompute(m,checker.hand);vertices={gid:v for gid,v,_ in cache}
    eligible=[m.geom(name).id for name in ('rf_tip','rf_ds_collision_1','rf_md_collision_1') if m.geom(name).id in vertices]
    measured_path=ROOT/'ring_reference_readonly_20261109_124719_945552/result.json';measured=json.loads(measured_path.read_text())
    measured_ticks={row['id']:row['position_ticks'] for row in measured['snapshots'][0]['motors']}
    taught=(np.array([measured_ticks[mid] for mid in np.array(SIM_TO_MOTOR_IDS)[rf]])-2048)*TICK
    seeds=[q0[rf],np.clip(taught,low[rf],high[rf])]
    targets=[]
    for axis in (0,2):
        for sign in (-1,1):
            for y in (6.,20.):
                for across in (-16.,0.,16.):
                    p=np.zeros(3);p[axis]=sign*27.3;p[1]=y;p[2 if axis==0 else 0]=across
                    targets.append(C+B@p/1000)
    folder.mkdir();began=time.perf_counter();fits=[];candidate_states=[]
    for gid in eligible:
        world=vertices[gid]@d.geom_xmat[gid].reshape(3,3).T+d.geom_xpos[gid]
        for target_index,target in enumerate(targets):
            local=vertices[gid][np.argmin(np.linalg.norm(world-target,axis=1))]
            for seed_index,seed in enumerate(seeds):
                proposal=restore(m,z)
                def residual(x):
                    proposal.qpos[:]=z['qpos'];proposal.qpos[rf]=x;proposal.ctrl[:]=proposal.qpos[:16]+lag
                    mujoco.mj_forward(m,proposal)
                    point=proposal.geom_xpos[gid]+proposal.geom_xmat[gid].reshape(3,3)@local
                    support,own=checker.contacts(proposal)
                    penetration=max((-float(c.dist)*1000 for c in proposal.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
                    return np.r_[(point-target)*1000,.05*(x-seed),max(own.values(),default=0.)*1000,
                                  max(support.values(),default=0.)*1000,max(0.,penetration-.8)*200]
                result=least_squares(residual,np.clip(seed,low[rf],high[rf]),bounds=(low[rf],high[rf]),max_nfev=60)
                error=float(np.linalg.norm(residual(result.x)[:3]));support,own=checker.contacts(proposal)
                contacts=[]
                for contact in proposal.contact:
                    a,b=int(contact.geom1),int(contact.geom2)
                    if cube not in (a,b):continue
                    name=m.geom(b if a==cube else a).name or ''
                    if name.startswith('rf_') and contact.dist<.0001:
                        contacts.append(dict(geom=name,distance_mm=float(contact.dist*1000)))
                goal=motor(proposal.ctrl)
                valid=bool(contacts and max(own.values(),default=0.)<=1e-6 and max(support.values(),default=0.)<=.15 and
                           np.all(goal>=checker.lo) and np.all(goal<=checker.hi) and error<2)
                row=dict(geom=m.geom(gid).name,target_index=target_index,seed=seed_index,point_error_mm=error,
                    RF_contacts=contacts,self_pairs_mm=own,static_candidate=valid,goal_ticks=goal.tolist())
                fits.append(row)
                if valid:candidate_states.append((error,goal.copy(),len(fits)-1))
        print(json.dumps(dict(geom=m.geom(gid).name,fits=len(fits),static_candidates=len(candidate_states))),flush=True)
    selected=[]
    for item in sorted(candidate_states,key=lambda row:row[0]):
        if all(np.max(abs(item[1]-old[1]))>20 for old in selected):selected.append(item)
        if len(selected)==4:break
    trials=[]
    for trial_index,(_,target,fit_index) in enumerate(selected):
        data=restore(m,z);guard=Guard(m);start=motor(data.ctrl)
        count=max(1,int(np.ceil(np.max(abs(target-start))/2)));goals=[];stop=[];rf_held=supported=samples=0
        for step in range(count+54):
            goal=np.rint(start+(target-start)*min(step+1,count)/count).astype(int)
            if np.max(abs(goal-(goals[-1] if goals else start)))>2+1e-9:raise AssertionError('Rate changed')
            data.ctrl[:]=ticks_to_sim(goal);goals.append(goal.copy())
            for _ in range(75):
                mujoco.mj_step(m,data);stop=guard.check(data)
                if np.linalg.norm(data.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if step>=count:
                    sample=sample_model(m,data,guard.hand,cube);samples+=1;supported+=sample['supported_sample']
                    rf_held+=sum(value['normal_N'] for name,value in sample['by_geom'].items() if name.startswith('rf_'))>.2
                if stop:break
            if stop:break
        support=sample_model(m,data,guard.hand,cube);angle=orientation_error_deg(data.geom_xmat[cube].reshape(3,3),B)
        shift=float(np.linalg.norm(data.geom_xpos[cube]-C)*1000)
        passed=bool(not stop and samples==4050 and rf_held/samples>.8 and supported/samples>.8 and angle<5 and shift<2)
        trials.append(dict(fit_index=fit_index,steps=len(goals),stop=stop,hold_samples=samples,
            rf_loaded_fraction=rf_held/max(1,samples),whole_hand_supported_fraction=supported/max(1,samples),
            cube_orientation_change_deg=angle,cube_center_change_mm=shift,temporary_RF_support_arrival_and_hold_passed=passed,
            terminal_support=support,worst=guard.worst,terminal_guard=guard.last,
            upper_screen_mm=min(value for _,value in gaps(m,data,data.geom_xpos[cube],data.geom_xmat[cube].reshape(3,3),cache,True))))
        np.savez_compressed(folder/f'trial{trial_index}_goals.npz',start=start,goals=goals)
        np.savez_compressed(folder/f'trial{trial_index}_end_state.npz',**{key:getattr(data,key) for key in BOUND},time=data.time,
                            up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        print(json.dumps({key:trials[-1][key] for key in ('steps','stop','rf_loaded_fraction','cube_orientation_change_deg','temporary_RF_support_arrival_and_hold_passed')}),flush=True)
    report=dict(status='RF_support_from_actual_supported_checkpoint1700',model_provenance=provenance,
        source_state_pool_sha256=sha(pool_path),recorded_RF_source_sha256=sha(measured_path),
        recorded_RF_used_as_bounded_seed_not_discarded_as_unreachable=True,
        cube_fixed_only_in_IK_proposals=True,cube_free_in_all_dynamic_arrivals=True,
        fits=fits,static_candidates=len(candidate_states),dynamic_trials=trials,
        temporary_RF_support_passed=any(row['temporary_RF_support_arrival_and_hold_passed'] for row in trials),
        first_rotation_or_next_rotation_passed=False,initialized_checkpoint_trials_not_full_cycle_success=True,
        original_native_guards_and_limits_unchanged=True,new_SAC_training_steps=0,
        devices_opened=0,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began)
    save_json(folder/'review.json',report)
    (folder/'source_search_supported_checkpoint_ring.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(dict(fits=len(fits),static_candidates=len(candidate_states),dynamic_trials=len(trials),temporary_RF_support_passed=report['temporary_RF_support_passed'])),flush=True)


if __name__=='__main__':main()
