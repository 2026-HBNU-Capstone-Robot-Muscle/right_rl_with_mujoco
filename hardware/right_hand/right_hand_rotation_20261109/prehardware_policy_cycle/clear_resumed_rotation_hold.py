"""Small finger-only sweep-margin correction after an executed next-turn branch.

The branch starts from an initialized MF/TH grip, not from the user's photo or
the first3015-goal path. Pose/cube states are never reset within actual trials.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from scipy.optimize import differential_evolution
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,TICK,SIM_TO_MOTOR_IDS,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'resumed_rotation_clear_hold'
    if folder.exists():raise RuntimeError('Preserve prior clearance correction')
    source=base/'thumb_branch_finish_checkpoint168/state.npz';failed=base/'thumb_branch_finish_to_reached_station/end_state.npz'
    reference=base/'initial_to_reached_workstation90/end_state.npz';z=dict(np.load(source,allow_pickle=False));ref=dict(np.load(reference,allow_pickle=False))
    model,initial,_,_,provenance=assets('active_ccd50');data=restore(model,z);guard=Guard(model);cube=model.geom('cube').id;C=data.geom_xpos[cube].copy();B=data.geom_xmat[cube].reshape(3,3).copy()
    certificate=SweepCertificate(model,guard.hand,.3);before,before_bad=certificate.query(data,C,B)
    badstate=restore(model,dict(np.load(failed,allow_pickle=False)));_,bad=certificate.query(badstate,badstate.geom_xpos[cube],badstate.geom_xmat[cube].reshape(3,3))
    names=[model.geom(int(certificate.ids[i])).name for i in bad];prefixes=sorted({(model.body(int(model.geom_bodyid[int(certificate.ids[i])])).name or '').split('_')[0] for i in bad})
    if not prefixes or any(prefix not in ('if','mf','rf','th') for prefix in prefixes):raise RuntimeError('No finger-only blocker: '+str(names))
    indices={'if':np.arange(1,4),'mf':np.arange(4,8),'rf':np.arange(8,12),'th':np.arange(12,16)}
    active=np.concatenate([indices[p] for p in prefixes]);sid=np.asarray(SIM_TO_MOTOR_IDS)-1;lag=data.ctrl-data.qpos[:16]
    low=np.maximum(guard.qlo,(np.ceil(guard.lo[sid])+2-2048)*TICK-lag);high=np.minimum(guard.qhi,(np.floor(guard.hi[sid])-2-2048)*TICK-lag)
    low=np.maximum(low,data.qpos[:16]-40*TICK);high=np.minimum(high,data.qpos[:16]+40*TICK)
    allgeom=precompute(model,guard.hand);geometry=[g for g in allgeom if any((model.body(int(model.geom_bodyid[g[0]])).name or '').startswith(p+'_') for p in prefixes)]
    folder.mkdir();(folder/'source_clear_resumed_rotation_hold.py').write_bytes(Path(__file__).read_bytes());proposals=[];rows=[];choices=[]
    print(json.dumps(dict(blocking_geoms=names,active_sim_joints=active.tolist(),source_lower_mm=before)),flush=True)
    for margin in (.7,1.1,1.6):
        ik=restore(model,z)
        def measure(x):
            ik.qpos[:]=z['qpos'];ik.qpos[active]=x;ik.ctrl[:]=ik.qpos[:16]+lag;mujoco.mj_forward(model,ik)
            support,own=guard.contacts(ik);upper=min(v for _,v in gaps(model,ik,C,B,geometry,True));pen=max((-float(c.dist)*1000 for c in ik.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            return upper,max(own.values(),default=0.),max(support.values(),default=0.),pen
        def objective(x):
            upper,own,support,pen=measure(x)
            return max(0.,margin-upper)*20+own*2000+support*1500+max(0.,pen-.8)*700+np.linalg.norm((x-data.qpos[active])/TICK)*.002
        fit=differential_evolution(objective,list(zip(low[active],high[active])),seed=int(margin*100),popsize=8,maxiter=90,x0=data.qpos[active],tol=1e-7,polish=True,workers=1)
        screen,own,support,pen=measure(fit.x);lower,bad=certificate.query(ik,C,B);desired=z['ctrl'].copy();desired[active]=ik.ctrl[active];goal=motor(desired)
        valid=bool(not bad and lower>.3 and own<=1e-6 and support<=.15 and pen<=1 and np.all(goal>=guard.lo) and np.all(goal<=guard.hi))
        proposals.append(dict(requested_screen_margin_mm=margin,exact_lower_mm=lower,static_valid=valid,nfev=int(fit.nfev)))
        if valid and all(np.max(abs(goal-g[0]))>1 for g in choices):choices.append((goal,margin))
    target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],180.);station=ref['qpos'][-7:-4]
    for k,(goal,margin) in enumerate(choices):
        actual=restore(model,z);check=Guard(model);start=motor(actual.ctrl);previous=start.copy();commands=[];trajectory=[actual.qpos.copy()];history=[];stop=[]
        count=max(1,int(np.ceil(np.max(abs(goal-start))/2)))
        for cycle in range(count+94):
            command=previous.copy();command[sid[active]]=np.rint(start[sid[active]]+(goal[sid[active]]-start[sid[active]])*min(cycle+1,count)/count)
            assert np.max(abs(command-previous))<=2+1e-9 and np.all(command>=check.lo) and np.all(command<=check.hi)
            actual.ctrl[active]=ticks_to_sim(command)[active];previous=command.copy();commands.append(command.copy())
            for _ in range(75):
                mujoco.mj_step(model,actual);stop=check.check(actual)
                if np.linalg.norm(actual.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if cycle>=count+40:
                    history.append(dict(qpos=actual.qpos.copy(),supported=sample_model(model,actual,check.hand,cube)['supported_sample']))
                if stop:break
            trajectory.append(actual.qpos.copy())
            if stop:break
        error=orientation_error_deg(actual.geom_xmat[cube].reshape(3,3),target);center_error=float(np.linalg.norm(actual.geom_xpos[cube]-station)*1000)
        fraction=sum(h['supported'] for h in history)/max(1,len(history));lower,bad=certificate.query(actual,actual.geom_xpos[cube],actual.geom_xmat[cube].reshape(3,3));minimum=lower;certified=0;drift=shift=0.
        if history:
            first=history[0]['qpos'];q0=first[-4:];C0=first[-7:-4]
            for h in history:
                shift=max(shift,float(np.linalg.norm(h['qpos'][-7:-4]-C0)*1000));drift=max(drift,float(np.degrees(2*np.arccos(np.clip(abs(float(h['qpos'][-4:]@q0)),0,1)))))
        pose_pass=bool(not stop and len(history)==4050 and fraction>.8 and error<5 and center_error<2 and shift<2 and drift<3 and face_up_error_deg(actual.geom_xmat[cube].reshape(3,3),5,initial['up'])<5)
        if pose_pass and not bad and lower>.3:
            geo=restore(model,z)
            for h in history:
                geo.qpos[:]=h['qpos'];mujoco.mj_forward(model,geo);value,failed=certificate.query(geo,geo.geom_xpos[cube],geo.geom_xmat[cube].reshape(3,3));minimum=min(minimum,value)
                if failed or value<=.3:break
                certified+=1
        passed=bool(pose_pass and certified==4050);sub=folder/f'trial{k}';sub.mkdir()
        np.savez_compressed(sub/'research_goals.npz',start=start,goals=commands);np.savez_compressed(sub/'trajectory.npz',qpos=trajectory,control_seconds=.075)
        np.savez_compressed(sub/'end_state.npz',**{key:getattr(actual,key) for key in BOUND},time=actual.time,up=initial['up'],initial_cube_rotation=initial['initial_cube_rotation'],initial_cube_center=initial['initial_cube_center'])
        row=dict(path=sub.relative_to(OUT).as_posix(),margin_objective_mm=margin,steps=len(commands),stop=stop,orientation_error_deg=error,reached_first_station_error_mm=center_error,
            hold_samples=len(history),whole_hand_support_fraction=fraction,upper_certified_samples=certified,upper_minimum_lower_mm=minimum,hold_shift_mm=shift,hold_drift_deg=drift,
            next_rotation_finish_and_hold_from_checkpoint_passed=passed,worst=check.worst,terminal_support=sample_model(model,actual,check.hand,cube))
        rows.append(row);save_json(sub/'review.json',row);print(json.dumps({name:row[name] for name in ('path','steps','stop','orientation_error_deg','reached_first_station_error_mm','upper_minimum_lower_mm','next_rotation_finish_and_hold_from_checkpoint_passed')}),flush=True)
    save_json(folder/'review.json',dict(status='small_clearance_correction_after_resumed_next_turn',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),target_station_reference=reference.relative_to(OUT).as_posix(),target_station_reference_sha256=sha(reference),
        blocking_geoms=names,free_sim_joints=active.tolist(),static_proposals=proposals,trials=rows,model_provenance=provenance,
        source_has_initialized_MF_TH_grip_ancestor=True,source_is_not_user_photo_pose=True,initial_to_first_to_next_cycle_not_certified=True,original_native_guards_and_limits_preserved=True,
        actual_cube_not_reset_within_trials=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
