"""Test a constant servo dwell at loaded prefixes of the actual handoff path."""
import json
import time
from pathlib import Path
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha
from cube_task_frame_20261105 import orientation_error_deg,upright_yaw_target
from handoff_support_wrench import contact_wrench


def state(d):return dict(**{key:getattr(d,key).copy() for key in BOUND},time=d.time)


def main():
    folder=OUT/'next_cycle/handoff_refinement_20261110/dwell_cutoff_search';folder.mkdir(exist_ok=True)
    if (folder/'review.json').exists():raise RuntimeError('Preserve completed dwell research')
    (folder/'source_search_load_dwell_cutoffs.py').write_bytes(Path(__file__).read_bytes())
    source=OUT/'next_cycle/handoff_integer_load_first_v6/end_state.npz';z=dict(np.load(source,allow_pickle=False))
    path=OUT/'next_cycle/handoff_integer_load_first_v7/research_goals.npz';recorded=np.load(path,allow_pickle=False)['goals']
    m,_,_,_,provenance=assets('active');d=restore(m,z);checker=Guard(m);cube=m.geom('cube').id
    loaded=[];began=time.perf_counter()
    for i,goal in enumerate(recorded):
        if np.max(abs(goal-motor(d.ctrl)))>2:raise AssertionError('Source goal spacing')
        d.ctrl[:]=ticks_to_sim(goal)
        for _ in range(75):
            mujoco.mj_step(m,d)
            if checker.check(d):raise AssertionError('Source path no longer reproduces guards')
        w=contact_wrench(m,d,checker.hand);forces=[w['by_geom'].get(n,{}).get('normal_N',0.) for n in ('rf_tip','th_tip','if_bs_collision_1')]
        if min(forces)>.2:loaded.append(dict(index=i,state=state(d),minimum_force=min(forces),forces=forces))
        if i>=80:break
    choices=sorted(loaded,key=lambda r:r['minimum_force'],reverse=True)[:12];rows=[];best=None
    for item in choices:
        origin=restore(m,item['state']);C0=origin.geom_xpos[cube].copy();B0=origin.geom_xmat[cube].reshape(3,3).copy();base=motor(origin.ctrl)
        for mode in ('hold','clear_middle1','clear_middle2','clear_middle_and_index'):
            goal=base.copy()
            if mode.startswith('clear_middle'):goal[9]+=1 if mode=='clear_middle1' else 2
            if mode=='clear_middle_and_index':goal[5]-=2
            check=Guard(m)
            if np.any(goal<check.lo) or np.any(goal>check.hi):continue
            trial=restore(m,item['state']);trial.ctrl[:]=ticks_to_sim(goal);count=good=0;stop=None;first_B=first_C=None;drift=shift=0.
            for t in range(10050):
                mujoco.mj_step(m,trial);stop=check.check(trial)
                if np.linalg.norm(trial.qpos[-7:-4]-z['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if t>=6000:
                    w=contact_wrench(m,trial,check.hand);count+=1
                    good+=all(w['by_geom'].get(n,{}).get('normal_N',0.)>.2 for n in ('rf_tip','th_tip','if_bs_collision_1'))
                    B=trial.geom_xmat[cube].reshape(3,3);C=trial.geom_xpos[cube]
                    if first_B is None:first_B=B.copy();first_C=C.copy()
                    drift=max(drift,orientation_error_deg(B,first_B));shift=max(shift,float(np.linalg.norm(C-first_C)*1000))
                if stop:break
            angle=orientation_error_deg(trial.geom_xmat[cube].reshape(3,3),B0);center=float(np.linalg.norm(trial.geom_xpos[cube]-C0)*1000)
            passed=not stop and count==4050 and good/count>.8 and angle<5 and center<2 and drift<3 and shift<2
            r=dict(prefix_goal=item['index']+1,mode=mode,source_prefix_forces_N=item['forces'],stop=stop,
                   tail_samples=count,support_fraction=good/max(1,count),relative_orientation_error_deg=angle,
                   relative_center_error_mm=center,hold_orientation_drift_deg=drift,hold_center_drift_mm=shift,
                   load_dwell_passed=bool(passed),worst=check.worst,goal_ticks=goal.tolist())
            rows.append(r)
            if passed and (best is None or r['prefix_goal']<best['row']['prefix_goal']):
                best=dict(row=r,expected_end=state(trial),goals=np.vstack([recorded[:item['index']+1],np.repeat(goal[None,:],134,axis=0)]))
        print(json.dumps(dict(prefix_goal=item['index']+1,comparisons=len(rows),passed=sum(r['load_dwell_passed'] for r in rows))),flush=True)
    result=dict(status='loaded_prefix_and_constant_servo_dwell_search',source_state=str(source.relative_to(OUT)),
        source_sha256=sha(source),source_goals=str(path.relative_to(OUT)),source_goals_sha256=sha(path),
        model_provenance=provenance,loaded_prefixes=len(loaded),tested_prefixes=len(choices),comparisons=rows,
        load_dwell_passed=best is not None,whole_cycle_ready_for_hardware=False,next90_rotation_passed=False,devices_opened=0)
    if best is not None:
        data=restore(m,z);check=Guard(m)
        for goal in best['goals']:
            if np.max(abs(goal-motor(data.ctrl)))>2:raise AssertionError('Selected dwell step exceeds two ticks')
            data.ctrl[:]=ticks_to_sim(goal)
            for _ in range(75):
                mujoco.mj_step(m,data)
                if check.check(data):raise AssertionError('Selected dwell failed persistent replay')
        difference={key:float(np.max(abs(getattr(data,key)-best['expected_end'][key]))) if best['expected_end'][key].size else 0. for key in BOUND}
        if max(difference.values())>1e-8:raise AssertionError(('Selected dwell seam mismatch',difference))
        np.savez_compressed(folder/'research_goals.npz',start=motor(z['ctrl']),goals=best['goals'])
        np.savez_compressed(folder/'end_state.npz',**state(data),up=z['up'],initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
        target=upright_yaw_target(z['initial_cube_rotation'],z['up'],90)
        result.update(selected=best['row'],goals=len(best['goals']),persistent_replay_alignment=difference,
                      original90_error_deg=orientation_error_deg(data.geom_xmat[cube].reshape(3,3),target),
                      actual_cube_not_reset_during_selected_prefix_and_dwell=True)
    result['elapsed_seconds']=time.perf_counter()-began;save_json(folder/'review.json',result)
    print(json.dumps({key:value for key,value in result.items() if key not in ('comparisons','model_provenance')}),flush=True)


if __name__=='__main__':main()
