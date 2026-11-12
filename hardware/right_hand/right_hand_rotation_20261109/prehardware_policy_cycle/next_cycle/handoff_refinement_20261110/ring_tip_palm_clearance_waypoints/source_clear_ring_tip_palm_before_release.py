"""Make a small RF-tip/palm gap before attempting the grip-release primitive."""
from pathlib import Path
import json
import numpy as np
import mujoco
from scipy.optimize import differential_evolution
from native_cycle import OUT,assets,restore,Guard,BOUND,TICK,SIM_TO_MOTOR_IDS,motor,ticks_to_sim,save_json,sha
from planning_pair_gap import PlanningPairGap
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'ring_tip_palm_clearance_waypoints'
    if folder.exists():raise RuntimeError('Preserve prior RF clearance study')
    source=base/'ring_thumb_turn_checkpoint/state.npz';z=dict(np.load(source,allow_pickle=False));model,initial,_,_,provenance=assets('active_ccd50');d=restore(model,z);guard=Guard(model)
    cube=model.geom('cube').id;tip=model.geom('rf_tip').id;pads=[model.geom('palm_collision_'+str(i)).id for i in (4,8)];pair=PlanningPairGap(model,[cube,tip]+pads)
    active=np.arange(8,12);sid=np.asarray(SIM_TO_MOTOR_IDS)-1;lag=d.ctrl-d.qpos[:16]
    low=np.maximum(guard.qlo,(np.ceil(guard.lo[sid])+2-2048)*TICK-lag);high=np.minimum(guard.qhi,(np.floor(guard.hi[sid])-2-2048)*TICK-lag)
    low=np.maximum(low,d.qpos[:16]-30*TICK);high=np.minimum(high,d.qpos[:16]+30*TICK)
    folder.mkdir();(folder/'source_clear_ring_tip_palm_before_release.py').write_bytes(Path(__file__).read_bytes());fits=[];choices=[];trials=[]
    for margin in (.2,.5,1.):
        ik=restore(model,z)
        def measure(x):
            ik.qpos[:]=z['qpos'];ik.qpos[active]=x;ik.ctrl[:]=ik.qpos[:16]+lag;mujoco.mj_forward(model,ik)
            support,own=guard.contacts(ik);gaps=[pair.gap(ik,tip,pad)*1000 for pad in pads];cube_gap=pair.gap(ik,tip,cube)*1000
            pen=max((-float(c.dist)*1000 for c in ik.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            return gaps,cube_gap,max(own.values(),default=0.),max(support.values(),default=0.),pen
        def objective(x):
            gaps,contact,own,support,pen=measure(x)
            return sum(max(0.,margin-g) for g in gaps)*20+abs(contact+.025)*4+own*2000+support*1000+max(0.,pen-.8)*500+np.linalg.norm((x-d.qpos[active])/TICK)*.003
        fit=differential_evolution(objective,list(zip(low[active],high[active])),seed=int(margin*100),popsize=10,maxiter=100,x0=np.clip(d.qpos[active],low[active],high[active]),tol=1e-8,polish=True,workers=1)
        gaps,contact,own,support,pen=measure(fit.x);desired=z['ctrl'].copy();desired[active]=ik.ctrl[active];goal=motor(desired)
        valid=bool(min(gaps)>.1 and contact<.2 and own<=1e-6 and support<=.15 and pen<=1 and np.all(goal>=guard.lo) and np.all(goal<=guard.hi))
        fits.append(dict(margin_mm=margin,separation_lower_bounds_mm=gaps,tip_cube_gap_screen_mm=contact,static_valid=valid,nfev=int(fit.nfev)))
        if valid and all(np.max(abs(goal-old[0]))>1 for old in choices):choices.append((goal,margin))
    for i,(goal,margin) in enumerate(choices):
        actual=restore(model,z);check=Guard(model);start=motor(actual.ctrl);previous=start.copy();commands=[];stop=[];count=max(1,int(np.ceil(np.max(abs(goal-start))/2)));supported=samples=0
        C0=actual.geom_xpos[cube].copy();B0=actual.geom_xmat[cube].reshape(3,3).copy()
        for step in range(count+30):
            command=previous.copy();command[sid[active]]=np.rint(start[sid[active]]+(goal[sid[active]]-start[sid[active]])*min(step+1,count)/count)
            assert np.max(abs(command-previous))<=2+1e-9 and np.all(command>=check.lo) and np.all(command<=check.hi)
            actual.ctrl[active]=ticks_to_sim(command)[active];previous=command.copy();commands.append(command.copy())
            for _ in range(75):
                mujoco.mj_step(model,actual);stop=check.check(actual)
                if np.linalg.norm(actual.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if step>=count:samples+=1;supported+=sample_model(model,actual,check.hand,cube)['supported_sample']
                if stop:break
            if stop:break
        gaps=[pair.gap(actual,tip,pad)*1000 for pad in pads];shift=float(np.linalg.norm(actual.geom_xpos[cube]-C0)*1000);angle=orientation_error_deg(actual.geom_xmat[cube].reshape(3,3),B0)
        passed=bool(not stop and samples==2250 and supported/2250>.8 and min(gaps)>.1)
        sub=folder/f'trial{i}';sub.mkdir();np.savez_compressed(sub/'research_goals.npz',start=start,goals=commands)
        np.savez_compressed(sub/'end_state.npz',**{key:getattr(actual,key) for key in BOUND},time=actual.time,up=initial['up'],initial_cube_rotation=initial['initial_cube_rotation'],initial_cube_center=initial['initial_cube_center'])
        row=dict(path=sub.relative_to(OUT).as_posix(),margin_objective_mm=margin,steps=len(commands),stop=stop,RF_palm_gap_lower_bounds_mm=gaps,whole_hand_support_fraction=supported/max(1,samples),samples=samples,
            cube_shift_mm=shift,orientation_change_deg=angle,RF_clear_supported_waypoint_passed=passed,worst=check.worst,terminal_guard=check.last)
        trials.append(row);save_json(sub/'review.json',row);print(json.dumps(row),flush=True)
    save_json(folder/'review.json',dict(status='RF_tip_palm_clearance_before_pressure_release',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),model_provenance=provenance,
        static_proposals=fits,trials=trials,other_finger_commands_unchanged=True,original_guards_and_bounds_preserved=True,intermediate_clearance_not_full_turn=True,
        new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
