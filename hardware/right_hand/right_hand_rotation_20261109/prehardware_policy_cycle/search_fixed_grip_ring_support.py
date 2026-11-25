"""RF/MF cooperation with IF/TH fixed; cube motion in IK is proposal only."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import mujoco
from scipy.optimize import differential_evolution
from scipy.spatial.transform import Rotation
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,TICK,SIM_TO_MOTOR_IDS,save_json,sha
from convex_distance_check_20261104 import CheckedDistance
from handoff_support_wrench import contact_wrench
from cube_task_frame_20261105 import orientation_error_deg,upright_yaw_target


def main():
    p=argparse.ArgumentParser();p.add_argument('--seed',type=int,default=0);p.add_argument('--iterations',type=int,default=180);a=p.parse_args()
    folder=OUT/'next_cycle/handoff_refinement_20261110'/f'fixed_grip_search{a.seed}';folder.mkdir(exist_ok=True)
    if (folder/'review.json').exists():raise RuntimeError('Preserve completed search')
    (folder/'source_search_fixed_grip_ring_support.py').write_bytes(Path(__file__).read_bytes())
    source=OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_schema_v2_confirmation/end_state.npz'
    z=dict(np.load(source,allow_pickle=False));m,_,_,_,provenance=assets('active');d=restore(m,z);guard=Guard(m);cube=m.geom('cube').id
    ids=[m.geom(name).id for name in ('if_tip','th_tip','if_bs_collision_1','rf_tip')];geometry=CheckedDistance(m,cube,ids)
    lag=z['ctrl']-z['qpos'][:16];sid=np.array(SIM_TO_MOTOR_IDS)-1
    qlo=np.maximum(guard.qlo,(np.ceil(guard.lo[sid])+2-2048)*TICK-lag)
    qhi=np.minimum(guard.qhi,(np.floor(guard.hi[sid])-2-2048)*TICK-lag)
    initial_rotation=Rotation.from_quat(z['qpos'][-4:][[1,2,3,0]])
    lower=np.r_[qlo[4:12],[-8.]*3,[-12.]*3];upper=np.r_[qhi[4:12],[8.]*3,[12.]*3]
    start=np.r_[z['qpos'][4:12],[0.]*6];start=np.clip(start,lower,upper)
    # The user's RF measurement is only a bounded search seed, never an issued
    # motor goal or an expanded joint limit.
    if a.seed%2:start[4:8]=np.clip((np.array([3053,2829,2961,3018])-2048)*TICK,qlo[8:12],qhi[8:12])
    began=time.perf_counter();last={};queries=0
    def assess(x):
        nonlocal last,queries
        queries+=1;d.qpos[:]=z['qpos'];d.qpos[4:12]=x[:8];d.qpos[-7:-4]+=x[8:11]/1000
        d.qpos[-4:]=(Rotation.from_rotvec(np.radians(x[11:14]))*initial_rotation).as_quat()[[3,0,1,2]]
        d.qvel[:]=0;d.ctrl[:]=d.qpos[:16]+lag;mujoco.mj_forward(m,d)
        support,own=guard.contacts(d);penetration=max((max(0.,-c.dist*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
        contact_gaps=np.array([geometry.gap(d,gid)*1000 for gid in ids])
        return contact_gaps,max(own.values(),default=0.),max(support.values(),default=0.),float(penetration)
    def objective(x):
        contact,self_pen,support,cube_pen=assess(x)
        return np.sum(np.abs(contact+.02))*250+self_pen*3000+support*3000+max(0.,cube_pen-.8)*2000+np.sum((x[:8]-start[:8])**2)*.02+np.linalg.norm(x[8:11])*.05+np.linalg.norm(x[11:14])*.02
    fit=differential_evolution(objective,list(zip(lower,upper)),seed=120+a.seed,x0=start,maxiter=a.iterations,popsize=10,tol=1e-8,polish=True)
    contact,self_pen,support,cube_pen=assess(fit.x);names=set()
    for c in d.contact:
        u,v=int(c.geom1),int(c.geom2)
        if cube in (u,v) and c.dist<.0004:names.add(m.geom(v if u==cube else u).name)
    valid={'if_tip','th_tip','if_bs_collision_1','rf_tip'}<=names and self_pen<=1e-6 and support<=.15 and cube_pen<=1
    proposal=dict(x=fit.x.tolist(),native_geometry_passed=bool(valid),contact_gaps_mm=contact.tolist(),
                  self_penetration_mm=self_pen,support_penetration_mm=support,cube_penetration_mm=cube_pen,contacts=sorted(names),queries=queries)
    np.savez_compressed(folder/'proposal.npz',qpos=d.qpos,ctrl=d.ctrl)
    dynamics=[]
    if valid:
        final=motor(d.ctrl);initial=motor(z['ctrl'])
        for order in ('middle_then_ring','coupled'):
            goals=[];previous=initial.copy()
            for indices in ([sid[4:8],sid[8:12]] if order=='middle_then_ring' else [sid[4:12]]):
                destination=previous.copy();destination[indices]=final[indices];n=max(1,int(np.ceil(np.max(abs(destination-previous))/2)))
                goals.extend(np.rint(previous+(destination-previous)*i/n).astype(int) for i in range(1,n+1));previous=destination
            goals.extend(previous.copy() for _ in range(200));data=restore(m,z);check=Guard(m);stop=None;count=good=0;first_B=first_C=None;drift=shift=0.
            for i,goal in enumerate(goals):
                data.ctrl[:]=ticks_to_sim(goal)
                for _ in range(75):
                    mujoco.mj_step(m,data);stop=check.check(data)
                    if np.linalg.norm(data.qpos[-7:-4]-z['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                    if i>=len(goals)-54:
                        count+=1;w=contact_wrench(m,data,check.hand)
                        good+=all(w['by_geom'].get(name,{}).get('normal_N',0.)>.2 for name in ('rf_tip','th_tip','if_bs_collision_1'))
                        B=data.geom_xmat[cube].reshape(3,3);C=data.geom_xpos[cube]
                        if first_B is None:first_B=B.copy();first_C=C.copy()
                        drift=max(drift,orientation_error_deg(B,first_B));shift=max(shift,float(np.linalg.norm(C-first_C)*1000))
                    if stop:break
                if stop:break
            target=upright_yaw_target(z['initial_cube_rotation'],z['up'],90);angle=orientation_error_deg(data.geom_xmat[cube].reshape(3,3),target)
            station=np.load(OUT/'baseline_archived_zero/end_state.npz',allow_pickle=False)['qpos'][-7:-4];center=float(np.linalg.norm(data.geom_xpos[cube]-station)*1000)
            passed=not stop and count==4050 and good/count>.8 and angle<5 and center<2 and drift<3 and shift<2
            branch=folder/order;branch.mkdir(exist_ok=True);np.savez_compressed(branch/'research_goals.npz',start=initial,goals=goals[:i+1])
            np.savez_compressed(branch/'end_state.npz',**{key:getattr(data,key) for key in BOUND},time=data.time,
                up=z['up'],initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
            dynamics.append(dict(order=order,steps=i+1,stop=stop,worst=check.worst,tail_1ms=count,support_fraction=good/max(1,count),target90_error_deg=angle,station_error_mm=center,passed=bool(passed)))
    report=dict(status='fixed_IF_TH_cooperative_RF_MF_contact_search',source_sha256=sha(source),source_state=str(source.relative_to(OUT)),
        model_provenance=provenance,proposal=proposal,dynamic_trials=dynamics,IF_TH_commands_fixed_during_motion=True,
        proposal_cube_transform_not_used_to_replace_actual_state=True,actual_cube_free_during_motion=True,
        original_constraints_retained=True,next90_rotation_passed=False,devices_opened=0,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began)
    save_json(folder/'review.json',report);print(json.dumps(report),flush=True)


if __name__=='__main__':main()
