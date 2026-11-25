"""Search bounded coupled goals whose constant servo command holds RF load.

Every branch starts from the same actual preserved handoff state, not an IK
pose. The result is a load-holding segment, never a second90-degree success.
"""
import json
import time
from pathlib import Path
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha
from cube_task_frame_20261105 import orientation_error_deg,upright_yaw_target
from handoff_support_wrench import contact_wrench


def main():
    folder=OUT/'next_cycle/handoff_refinement_20261110/stable_load_search';folder.mkdir(exist_ok=True)
    if (folder/'review.json').exists():raise RuntimeError('Preserve completed research')
    (folder/'source_search_stable_load_hold.py').write_bytes(Path(__file__).read_bytes())
    source=OUT/'next_cycle/handoff_integer_load_first_v6/end_state.npz';z=dict(np.load(source,allow_pickle=False))
    m,_,_,_,provenance=assets('active');d=restore(m,z);base=motor(d.ctrl);cube=m.geom('cube').id
    C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy();rng=np.random.default_rng(261110)
    active=np.array([0,1,2,3,12,13,14,15]);changes=[np.zeros(16,dtype=int)]
    for i in range(256):
        v=np.zeros(16,dtype=int);v[active]=rng.choice([-2,0,2] if i<128 else [-2,2],8)
        v[5]=rng.choice([-2,0]);v[9]=rng.choice([0,2]);changes.append(v)
    changes=np.unique(changes,axis=0);rows=[];started=time.perf_counter()

    def run(goal,n,tail):
        data=restore(m,z);check=Guard(m);data.ctrl[:]=ticks_to_sim(goal);stop=None;count=good=0;minimum=np.full(3,np.inf);sum_force=np.zeros(3)
        first_B=first_C=None;drift=shift=0.
        for step in range(n):
            mujoco.mj_step(m,data);stop=check.check(data)
            if np.linalg.norm(data.qpos[-7:-4]-z['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            if step>=n-tail:
                w=contact_wrench(m,data,check.hand);f=np.array([w['by_geom'].get(name,{}).get('normal_N',0.) for name in ('rf_tip','th_tip','if_bs_collision_1')])
                count+=1;good+=bool(np.all(f>.2));minimum=np.minimum(minimum,f);sum_force+=f
                B=data.geom_xmat[cube].reshape(3,3);C=data.geom_xpos[cube]
                if first_B is None:first_B=B.copy();first_C=C.copy()
                drift=max(drift,orientation_error_deg(B,first_B));shift=max(shift,float(np.linalg.norm(C-first_C)*1000))
            if stop:break
        angle=orientation_error_deg(data.geom_xmat[cube].reshape(3,3),B0);center=float(np.linalg.norm(data.geom_xpos[cube]-C0)*1000)
        r=dict(steps_1ms=step+1,stop=stop,tail_samples=count,support_fraction=good/max(1,count),
               mean_RF_TH_base_N=(sum_force/max(1,count)).tolist(),minimum_RF_TH_base_N=minimum.tolist() if count else None,
               relative_orientation_error_deg=angle,relative_center_error_mm=center,
               tail_orientation_drift_deg=drift,tail_center_drift_mm=shift,worst=check.worst)
        return data,r

    checker=Guard(m)
    for i,delta in enumerate(changes):
        goal=base+delta
        if np.any(goal<checker.lo) or np.any(goal>checker.hi):continue
        data,r=run(goal,750,250);r.update(index=i,delta_ticks=delta.tolist(),goal_ticks=goal.tolist());rows.append(r)
        if (i+1)%32==0:print(json.dumps(dict(branches=i+1,best_load_fraction=max((v['support_fraction'] for v in rows),default=0.))),flush=True)
    viable=[r for r in rows if not r['stop'] and r['relative_orientation_error_deg']<5 and r['relative_center_error_mm']<2]
    ranked=sorted(viable,key=lambda r:(r['support_fraction'],min(r['mean_RF_TH_base_N'])),reverse=True)[:4];qualified=[]
    for index,candidate in enumerate(ranked):
        goal=np.array(candidate['goal_ticks']);data,r=run(goal,10050,4050)
        passed=not r['stop'] and r['tail_samples']==4050 and r['support_fraction']>.8 and r['relative_orientation_error_deg']<5 and r['relative_center_error_mm']<2 and r['tail_orientation_drift_deg']<3 and r['tail_center_drift_mm']<2
        target=folder/f'candidate{index}';target.mkdir(exist_ok=True)
        count=int(np.ceil(r['steps_1ms']/75))
        np.savez_compressed(target/'research_goals.npz',start=base,goals=np.repeat(goal[None,:],count,axis=0))
        np.savez_compressed(target/'end_state.npz',**{key:getattr(data,key) for key in BOUND},time=data.time,
            up=z['up'],initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
        original_target=upright_yaw_target(z['initial_cube_rotation'],z['up'],90)
        r.update(candidate=candidate,load_holding_segment_passed=bool(passed),
                 original90_orientation_error_deg=orientation_error_deg(data.geom_xmat[cube].reshape(3,3),original_target),
                 source_state_sha256=sha(source),one_initial_goal_change_max2ticks=True,
                 whole_cycle_success=False,devices_opened=0)
        save_json(target/'review.json',r);qualified.append(dict(path=str(target.relative_to(OUT)),**r))
    report=dict(status='bounded_coupled_stable_load_search',source_state=str(source.relative_to(OUT)),source_sha256=sha(source),
        model_provenance=provenance,screen_trials=rows,qualified=qualified,
        any_load_holding_segment_passed=any(r['load_holding_segment_passed'] for r in qualified),
        initialized_at_preserved_actual_handoff_state_not_IK=True,
        original90_pose_and_working_upper_space_not_yet_certified=True,
        next90_rotation_passed=False,hardware_execution_allowed=False,devices_opened=0,elapsed_seconds=time.perf_counter()-started)
    save_json(folder/'review.json',report)
    print(json.dumps(dict(qualified=qualified,elapsed_seconds=report['elapsed_seconds'])),flush=True)


if __name__=='__main__':main()
