"""Find room for a reachable support finger, then validate staged free motion."""
import argparse
import time
import json
import numpy as np
import mujoco
from scipy.optimize import differential_evolution
from native_cycle import OUT,assets,restore,Guard,motor,ticks_to_sim,BOUND,TICK,SIM_TO_MOTOR_IDS,save_json,sha
from cube_task_frame_20261105 import orientation_error_deg
from inspect_endpoints import inspect
from convex_distance_check_20261104 import CheckedDistance


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--finger',choices=('rf','mf'),default='rf')
    parser.add_argument('--fixed-index',action='store_true');parser.add_argument('--static-only',action='store_true')
    parser.add_argument('--keep-index-contact',action='store_true')
    a=parser.parse_args()
    if a.fixed_index and a.keep_index_contact:raise ValueError('Choose one index constraint')
    suffix='_fixed_index' if a.fixed_index else ('_keep_index_contact' if a.keep_index_contact else '')
    folder=OUT/'next_cycle'/('clearance_'+a.finger+suffix);folder.mkdir(parents=True,exist_ok=True)
    if (folder/'review.json').exists():raise RuntimeError('Preserve completed run')
    from pathlib import Path
    (folder/'source_clear_support_contact_mode.py').write_bytes(Path(__file__).read_bytes())
    source=OUT/'connected_sac_and_teacher/first_quarter_end_state.npz'
    workspace=OUT/'next_cycle'/('workspace_'+a.finger+'_checked')
    reach=json.loads((workspace/'review.json').read_text());seeds=[r for r in reach['cases'] if r['pure_kinematic_contact_found']]
    m,_,_,_,provenance=assets('active');z=dict(np.load(source,allow_pickle=False));d=restore(m,z);guard=Guard(m)
    fixed=np.arange(8,12) if a.finger=='rf' else np.arange(4,8)
    other=np.arange(4,8) if a.finger=='rf' else np.arange(8,12)
    free=other if a.fixed_index else np.r_[np.arange(1,4),other];sid=np.array(SIM_TO_MOTOR_IDS)-1
    lag=d.ctrl-d.qpos[:16];qlo=np.maximum(guard.qlo,(np.ceil(guard.lo[sid])+2-2048)*TICK-lag)
    qhi=np.minimum(guard.qhi,(np.floor(guard.hi[sid])-2-2048)*TICK-lag)
    cube=m.geom('cube').id;tip=m.geom(a.finger+'_tip').id;B=d.geom_xmat[cube].reshape(3,3).copy()
    index_geometry=CheckedDistance(m,cube,[m.geom('if_tip').id])
    began=time.perf_counter();proposals=[];snapshots={}
    for row in seeds:
        base=z['qpos'].copy();base[fixed]=row['sim_angles']
        def evaluate(x):
            d.qpos[:]=base;d.qpos[free]=x;d.ctrl[:]=d.qpos[:16]+lag;d.qvel[:]=0;mujoco.mj_forward(m,d)
            support,own=guard.contacts(d)
            pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            return support,own,pen
        def objective(x):
            support,own,pen=evaluate(x)
            # Only IK cost weights change; final acceptance uses original guards.
            contact_error=abs(index_geometry.gap(d,m.geom('if_tip').id)*1000+.02) if a.keep_index_contact else 0.
            return max(own.values(),default=0.)*1000+max(support.values(),default=0.)*1000+max(0.,pen-.8)*1000+.001*np.sum((x-z['qpos'][free])**2)+contact_error*1000
        fit=differential_evolution(objective,list(zip(qlo[free],qhi[free])),seed=row['seed']+20,
                                   x0=np.clip(z['qpos'][free],qlo[free],qhi[free]),maxiter=280,popsize=12,tol=1e-8,polish=True)
        support,own,pen=evaluate(fit.x);names=set()
        for c in d.contact:
            x,y=int(c.geom1),int(c.geom2)
            if cube in (x,y) and c.dist<.0004:names.add(m.geom(y if x==cube else x).name)
        needed={a.finger+'_tip','th_tip','if_bs_collision_1'}|({'if_tip'} if (a.fixed_index or a.keep_index_contact) else set())
        valid=needed<=names and max(own.values(),default=0.)<=1e-6 and max(support.values(),default=0.)<=.15 and pen<=1.
        proposal=dict(seed=row['seed'],static_geometry_passed=bool(valid),self_pairs_mm=own,
                      support_pairs_mm=support,cube_penetration_mm=pen,contacts=sorted(names),
                      squared_joint_change_rad2=float(np.sum((d.qpos[:16]-z['qpos'][:16])**2)),nfev=int(fit.nfev))
        proposals.append(proposal)
        if valid:
            snapshots[row['seed']]=d.qpos[:16].copy()
            np.savez_compressed(folder/f'proposal{row["seed"]}.npz',qpos=d.qpos,ctrl=d.ctrl)
        print(json.dumps(dict(seed=row['seed'],static_geometry_passed=bool(valid),self_mm=max(own.values(),default=0.),contacts=sorted(names))),flush=True)
    dynamics=[]
    for seed,pose in ([] if a.static_only else snapshots.items()):
        # Compute a bounded position-target preload hypothesis on the new finger.
        d.qpos[:]=z['qpos'];d.qpos[:16]=pose;d.ctrl[:]=pose+lag;d.qvel[:]=0;mujoco.mj_forward(m,d)
        contact=next(c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,tip})
        normal=contact.frame[:3]*(1 if int(contact.geom1)==cube else -1)
        J=np.zeros((3,m.nv));mujoco.mj_jac(m,d,J,None,contact.pos,int(m.geom_bodyid[tip]))
        closing=-J[:,:16].T@normal/m.actuator_gainprm[:,0]
        for preload in (0.,.3,.7):
            for order in ('other_then_new_then_index','other_then_new_and_index'):
                target=pose+lag;target[fixed]+=closing[fixed]*preload;final=motor(target)
                if np.any(final<guard.lo) or np.any(final>guard.hi):continue
                data=restore(m,z);checker=Guard(m);start=motor(data.ctrl);previous=start.copy();rows=[];phases=[]
                groups=[other,fixed,np.arange(1,4)] if order=='other_then_new_then_index' else [other,np.r_[fixed,np.arange(1,4)]]
                for group in groups:
                    goal=previous.copy();goal[sid[group]]=final[sid[group]]
                    count=max(1,int(np.ceil(np.max(abs(goal-previous))/2)))
                    rows.extend(np.rint(previous+(goal-previous)*(i+1)/count).astype(int) for i in range(count));phases.extend(['transition']*count)
                    previous=goal
                rows.extend(final.copy() for _ in range(134));phases.extend(['hold']*134)
                stop=None;held=good=0;hold_B=hold_C=None;hold_angle=hold_shift=0.
                for k,goal in enumerate(rows):
                    data.ctrl[:]=ticks_to_sim(goal)
                    for _ in range(75):
                        mujoco.mj_step(m,data);stop=checker.check(data)
                        if np.linalg.norm(data.qpos[-7:-4]-z['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                        if k>=len(rows)-54:
                            force={}
                            for ci,c in enumerate(data.contact):
                                x,y=int(c.geom1),int(c.geom2)
                                if cube not in (x,y):continue
                                name=m.geom(y if x==cube else x).name
                                if name in needed:
                                    f=np.zeros(6);mujoco.mj_contactForce(m,data,ci,f);force[name]=force.get(name,0.)+float(f[0])
                            held+=1;good+=all(force.get(n,0.)>.2 for n in needed)
                            current_B=data.geom_xmat[cube].reshape(3,3);current_C=data.geom_xpos[cube]
                            if hold_B is None:hold_B=current_B.copy();hold_C=current_C.copy()
                            hold_angle=max(hold_angle,orientation_error_deg(current_B,hold_B))
                            hold_shift=max(hold_shift,float(np.linalg.norm(current_C-hold_C)*1000))
                        if stop:break
                    if stop:break
                label=f'seed{seed}_preload{preload:g}_{order}';trial=folder/label;trial.mkdir(exist_ok=True)
                np.savez_compressed(trial/'research_goals.npz',start=start,goals=rows[:k+1])
                np.savez_compressed(trial/'end_state.npz',**{key:getattr(data,key) for key in BOUND},time=data.time,
                    up=z['up'],initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
                endpoint=inspect(m,data,z['up'],z['initial_cube_rotation'])
                orientation_drift=orientation_error_deg(data.geom_xmat[cube].reshape(3,3),B)
                center_error=float(np.linalg.norm(data.qpos[-7:-4]-z['qpos'][-7:-4])*1000)
                report=dict(seed=seed,preload_force_hypothesis_N=preload,order=order,goals_completed=k+1,
                            planned_goals=len(rows),stop=stop,worst=checker.worst,endpoint=endpoint,
                            hold_samples=held,three_support_fraction=good/max(1,held),
                            orientation_drift_deg=orientation_drift,center_error_mm=center_error,
                            hold_orientation_drift_deg=hold_angle,hold_center_drift_mm=hold_shift,
                            third_support_passed=not stop and held==4050 and good/4050>.8 and
                                orientation_drift<5 and center_error<2 and hold_angle<3 and hold_shift<2,
                            no_cube_or_hand_state_reset_during_suffix=True)
                save_json(trial/'review.json',report);dynamics.append(dict(path=trial.relative_to(OUT).as_posix(),**report))
                print(json.dumps(dict(trial=label,stop=stop,goals_completed=k+1,third_support_passed=report['third_support_passed'])),flush=True)
    report=dict(status='cooperative_clearance_and_staged_free_support_trials',finger=a.finger,
        source_sha256=sha(source),workspace_report_sha256=sha(workspace/'review.json'),model_provenance=provenance,
        other_fingers_optimized_to_clear_new_support=True,index_joints_fixed=a.fixed_index,
        index_contact_required=a.keep_index_contact,
        static_only=a.static_only,native_limits_and_model_unchanged=True,
        static_proposals=proposals,dynamic_trials=dynamics,second90_rotation_passed=False,
        devices_opened=0,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began)
    save_json(folder/'review.json',report)


if __name__=='__main__':main()
