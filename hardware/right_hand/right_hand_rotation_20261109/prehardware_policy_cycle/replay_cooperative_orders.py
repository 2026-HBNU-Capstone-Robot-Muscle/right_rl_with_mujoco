"""Change the order of a real state continuation, with no cube pose reset."""
import argparse
import time
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,motor,ticks_to_sim,BOUND,SIM_TO_MOTOR_IDS,save_json,sha
from cube_task_frame_20261105 import orientation_error_deg
from inspect_endpoints import inspect


def ramp(start,end):
    count=max(1,int(np.ceil(np.max(np.abs(end-start))/2)))
    return [np.rint(start+(end-start)*(i+1)/count).astype(int) for i in range(count)]


def main():
    p=argparse.ArgumentParser();p.add_argument('--seeds',type=int,nargs='+',default=[0,1]);a=p.parse_args()
    folder=OUT/'next_cycle/ordered_rf_handoff';folder.mkdir(parents=True,exist_ok=True)
    if (folder/'review.json').exists():raise RuntimeError('Preserve completed trial family')
    m,_,_,_,provenance=assets('active');source=OUT/'connected_sac_and_teacher/first_quarter_end_state.npz'
    z=dict(np.load(source,allow_pickle=False));d=restore(m,z);initial=motor(d.ctrl);lag=d.ctrl-d.qpos[:16]
    cube=m.geom('cube').id;tip=m.geom('rf_tip').id;B0=d.geom_xmat[cube].reshape(3,3).copy()
    C0=d.geom_xpos[cube].copy();sid=np.array(SIM_TO_MOTOR_IDS)-1;records=[];began=time.perf_counter()
    for seed in a.seeds:
        static=np.load(OUT/f'next_cycle/clearance_rf/proposal{seed}.npz');pose=static['qpos'][:16]
        d.qpos[:]=static['qpos'];d.ctrl[:]=pose+lag;d.qvel[:]=0;mujoco.mj_forward(m,d)
        contact=next(c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,tip})
        normal=contact.frame[:3]*(1 if int(contact.geom1)==cube else -1);J=np.zeros((3,m.nv))
        mujoco.mj_jac(m,d,J,None,contact.pos,int(m.geom_bodyid[tip]))
        preload=-J[:,:16].T@normal/m.actuator_gainprm[:,0]*.3
        command=pose+lag;command[8:12]+=preload[8:12];target=motor(command)
        index=sid[1:4];middle=sid[4:8];ring=sid[8:12]
        for order in ('index_middle_ring','index_and_middle_ring','all_together',
                      'middle_half_index_rest','ring_half_index_middle_rest','index_half_middle_rest'):
            points=[];previous=initial.copy()
            def add(group,fraction=1.):
                nonlocal previous
                nxt=previous.copy();nxt[group]=np.rint(initial[group]+(target[group]-initial[group])*fraction).astype(int)
                points.append(nxt);previous=nxt
            if order=='index_middle_ring':add(index);add(middle);add(ring)
            elif order=='index_and_middle_ring':add(np.r_[index,middle]);add(ring)
            elif order=='all_together':add(np.r_[index,middle,ring])
            elif order=='middle_half_index_rest':add(middle,.5);add(index);add(np.r_[middle,ring])
            elif order=='ring_half_index_middle_rest':add(ring,.5);add(np.r_[index,middle]);add(ring)
            else:add(index,.5);add(middle);add(np.r_[index,ring])
            rows=[];boundaries=[];previous=initial.copy()
            for nxt in points:
                rows.extend(ramp(previous,nxt));boundaries.append(len(rows));previous=nxt
            rows.extend(target.copy() for _ in range(134));checker=Guard(m);data=restore(m,z)
            if np.any(target<checker.lo) or np.any(target>checker.hi):continue
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
                            if name in ('if_bs_collision_1','rf_tip','th_tip'):
                                f=np.zeros(6);mujoco.mj_contactForce(m,data,ci,f);force[name]=force.get(name,0.)+float(f[0])
                        held+=1;good+=all(force.get(n,0.)>.2 for n in ('if_bs_collision_1','rf_tip','th_tip'))
                        B=data.geom_xmat[cube].reshape(3,3);C=data.geom_xpos[cube]
                        if hold_B is None:hold_B=B.copy();hold_C=C.copy()
                        hold_angle=max(hold_angle,orientation_error_deg(B,hold_B));hold_shift=max(hold_shift,float(np.linalg.norm(C-hold_C)*1000))
                    if stop:break
                if stop:break
            endpoint=inspect(m,data,z['up'],z['initial_cube_rotation'])
            error=orientation_error_deg(data.geom_xmat[cube].reshape(3,3),B0)
            center=float(np.linalg.norm(data.geom_xpos[cube]-C0)*1000)
            passed=not stop and held==4050 and good/4050>.8 and error<5 and center<2 and hold_angle<3 and hold_shift<2
            trial=folder/f'seed{seed}_{order}';trial.mkdir(exist_ok=True)
            np.savez_compressed(trial/'research_goals.npz',start=initial,goals=rows[:k+1])
            np.savez_compressed(trial/'end_state.npz',**{key:getattr(data,key) for key in BOUND},time=data.time,
                                up=z['up'],initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
            report=dict(seed=seed,order=order,preload_force_hypothesis_N=.3,goals_completed=k+1,
                        planned_goals=len(rows),stage_boundaries=boundaries,stop=stop,worst=checker.worst,
                        endpoint=endpoint,hold_1ms_samples=held,three_support_fraction=good/max(1,held),
                        orientation_drift_deg=error,center_error_mm=center,hold_yaw_drift_deg=hold_angle,
                        hold_center_drift_mm=hold_shift,temporary_third_support_passed=passed,
                        second90_rotation_passed=False,source_sha256=sha(source),model_provenance=provenance,
                        devices_opened=0,hardware_execution_allowed=False)
            save_json(trial/'review.json',report);records.append(dict(path=trial.relative_to(OUT).as_posix(),**report))
            print(json.dumps({key:report[key] for key in ('seed','order','goals_completed','stop','orientation_drift_deg','temporary_third_support_passed')}),flush=True)
    save_json(folder/'review.json',dict(status='ordered_cooperative_contact_transition_trials',trials=records,
        passed_transient_supports=sum(r['temporary_third_support_passed'] for r in records),
        devices_opened=0,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began))


if __name__=='__main__':main()
